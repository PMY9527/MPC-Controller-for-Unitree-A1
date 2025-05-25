#include "FSM/State_MPC.h"

/* THIS SCRIPT IS FOR MPC CONTROL OF UNITREE A1 ROBOT USING MIT CHEETAH'S SINGLE RIGID BODY MODEL. 
    === State Vector ===    === QP Solver ===          ======= Tested Benchmarks (Gazebo)======= 
     0. 滚转角 Φ (roll)          Quadprogpp                 ~ 300 HZ at 10 Prediction Horizon 
     1. 俯仰角 θ (pitch)                               Climbing stairs at 5 cm height and 20 cm length
     2. 偏航角 ψ (yaw)                                                Top speed 1.7 m/s
     3. x CoM
     4. y CoM
     5. z CoM
     6. dΦ (roll rate)
     7. dθ (pitch rate)
     8. dψ (yaw rate)
     9. dx CoM
     10. dy CoM         === AUTHOR: MINGYANG PAN ===
     11. dz CoM
     12. -g
*/

State_MPC::State_MPC(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::MPC, "mpc_quadprogpp"),
      _est(ctrlComp->estimator), _phase(ctrlComp->phase),
      _contact(ctrlComp->contact), _robModel(ctrlComp->robotModel),
      _balCtrl(ctrlComp->balCtrl)
{
    _gait = new GaitGenerator(ctrlComp); 
    _gaitHeight = 0.1; // 抬腿高度设置 gait height setting

    // unitree A1 
    _Kpp = Vec3(20, 20, 100).asDiagonal(); 
    _Kdp = Vec3(20, 20, 20).asDiagonal();
    _kpw = 400;
    _Kdw = Vec3(50, 50, 50).asDiagonal();
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();

    _vxLim = _robModel->getRobVelLimitX();
    _vyLim = _robModel->getRobVelLimitY();
    _wyawLim = _robModel->getRobVelLimitYaw();

    _mass = _robModel->getRobMass();
    Ic = _robModel->getRobInertial();
    miuMat << 1, 0, miu,
             -1, 0, miu,
             0,  1, miu,
             0, -1, miu,
             0,  0,  -1;

    setWeight();
}

void State_MPC::setWeight() // Setting up Q and R matrices for MPC.
{
    Q_diag.resize(1, nx);
    R_diag.resize(1, nu);
    Q_diag.setZero();
    R_diag.setZero();

    Q_diag << 30.0, 30.0, 1.0, // r,p,y, 
            1.0, 1.0, 220.0,   // pCoM
            1.05, 1.05, 1.05,  // w
            20.0, 20.0, 20.0,  // vcom 
            0.0;
            
    R_diag <<   1.0, 1.0, 0.1, 
                1.0, 1.0, 0.1, 
                1.0, 1.0, 0.1,  
                1.0, 1.0, 0.1; 
    R_diag = R_diag * 1e-5; 

    Q_diag_N.resize(1, nx * mpc_N);
    R_diag_N.resize(1, nu * mpc_N);
    Q_diag_N.setZero();
    R_diag_N.setZero();
    Q.resize(nx * mpc_N, nx * mpc_N);
    R.resize(nu * mpc_N, nu * mpc_N);
    Q.setZero();
    R.setZero();

    for (int i = 0; i < mpc_N; i++)
    {
        Q_diag_N.block<1, nx>(0, i * nx) = Q_diag;
    }

    for (int i = 0; i < mpc_N; i++)
    {
        R_diag_N.block<1, nu>(0, i * nu) = R_diag;
    }

    for (int i = 0; i < nx * mpc_N; i++)
    {
        Q(i, i) = Q_diag_N(0, i);
    }

    for (int i = 0; i < nu * mpc_N; i++)
    {
        R(i, i) = R_diag_N(0, i);
    }
}

State_MPC::~State_MPC()
{
    delete _gait;
}

void State_MPC::enter()
{   
    _pcd = _est->getPosition();
    _pcd(2) = -_robModel->getFeetPosIdeal()(2, 0);
    _vCmdBody.setZero();
    _yawCmd = _lowState->getYaw();
    _Rd = rotz(_yawCmd);
    _wCmdGlobal.setZero();
    _ctrlComp->ioInter->zeroCmdPanel();
    _gait->restart();                
}

void State_MPC::exit()
{
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();
}

FSMStateName State_MPC::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B || (_forceFeetGlobal.array() != _forceFeetGlobal.array()).any()) // if nan, quit to passive for debugging
    {
        return FSMStateName::PASSIVE;
    }
    else if (_lowState->userCmd == UserCommand::L2_A)
    {
        return FSMStateName::FIXEDSTAND;
    }
    else
    {
        return FSMStateName::MPC;
    }
}

void State_MPC::run()
{   
    //static auto last_time = std::chrono::high_resolution_clock::now(); 

    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();
    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();
    _rpy =_lowState->getRPY();
    _yaw = _lowState->getYaw();
    _dYaw = _lowState->getDYaw();
    
    _userValue = _lowState->userValue;

    getUserCmd();
    calcCmd();

    _gait->setGait(_vCmdGlobal.segment(0, 2), _wCmdGlobal(2), _gaitHeight);
    _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal);

    calcTau();  // SetMatrices() --> Publishing() --> ConstraintsSetup() --> solveQP() --> calcFe() --> calcTau()
    calcQQd(); // q and qd

    _ctrlComp->setStartWave();
    _lowCmd->setTau(_tau);
    _lowCmd->setQ(vec34ToVec12(_qGoal));
    _lowCmd->setQd(vec34ToVec12(_qdGoal));
    
    for (int i(0); i < 4; ++i)
    {
        if ((*_contact)(i) == 0)
        {
            _lowCmd->setSwingGain(i);
        }
        else
        {
            _lowCmd->setStableGain(i);
        }
    }
    /*
    auto current_time = std::chrono::high_resolution_clock::now();
    dt_actual = std::chrono::duration<double>(current_time - last_time).count();
    last_time = current_time;
    ROS_INFO("Actual dt: %.4f s", dt_actual); // Benchmarking -- time for each iteration. For mpc_N = 5, dt = 1.8 ~ 2.3 ms. 
    */
}

void State_MPC::setHighCmd(double vx, double vy, double wz)
{
    _vCmdBody(0) = vx;
    _vCmdBody(1) = vy;
    _vCmdBody(2) = 0;
    _dYawCmd = wz;
}

void State_MPC::getUserCmd()
{
    /* Movement */
    _vCmdBody(0) = invNormalize(_userValue.ly, -speed_limx, speed_limx);
    //_vCmdBody(0) = invNormalize(_userValue.ly, _vxLim(0), _vxLim(1)); // +_0.4
    _vCmdBody(1) = -invNormalize(_userValue.lx, _vyLim(0), _vyLim(1));
    _vCmdBody(2) = 0;

    /* Turning */
    _dYawCmd = -invNormalize(_userValue.rx, _wyawLim(0), _wyawLim(1));
    _dYawCmd = 0.9 * _dYawCmdPast + (1 - 0.9) * _dYawCmd;
    _dYawCmdPast = _dYawCmd;
}

void State_MPC::calcCmd() 
{
    /* Movement */
    _vCmdGlobal = _B2G_RotMat * _vCmdBody;

    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0) - 0.2, _velBody(0) + 0.2));
    _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_velBody(1) - 0.2, _velBody(1) + 0.2));

    _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * d_time, Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05));
    _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * d_time, Vec2(_posBody(1) - 0.05, _posBody(1) + 0.05));
    _pcd(2) = -_robModel->getFeetPosIdeal()(2, 0);
    _vCmdGlobal(2) = 0;

    /* Turning */
    _yawCmd = _yawCmd + _dYawCmd * d_time;

    _Rd = rotz(_yawCmd);
    _wCmdGlobal(2) = _dYawCmd;
}

void State_MPC::calcTau() // Calculate joint torques based on contact forces solved via MPC. 
{
    calcFe();
    
    //std::cout << "********forceFeetGlobal(MPC)********" << std::endl
    //         << _forceFeetGlobal << std::endl;
    
    for (int i(0); i < 4; ++i)
    {
        if ((*_contact)(i) == 0) // Swing Legs
        { 
            _forceFeetGlobal.col(i) = _KpSwing * (_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing * (_velFeetGlobalGoal.col(i) - _velFeetGlobal.col(i));
        }
    }

    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;
    _q = vec34ToVec12(_lowState->getQ());
    _tau = _robModel->getTau(_q, _forceFeetBody);

}

void State_MPC::calcQQd() // Desired joint angles and angular velocities. 
{

    Vec34 _posFeet2B;
    _posFeet2B = _robModel->getFeet2BPositions(*_lowState, FrameType::BODY);

    for (int i(0); i < 4; ++i)
    {
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody);
    }

    _qGoal = vec12ToVec34(_robModel->getQ(_posFeet2BGoal, FrameType::BODY)); // 关节的目标角度
    _qdGoal = vec12ToVec34(_robModel->getQd(_posFeet2B, _velFeet2BGoal, FrameType::BODY)); // 关节的目标角速度
}

#undef inverse

void State_MPC::SetMatrices() // Setting up Hessian, G and Gradient, g0. 
{
    /*
    Computes desired states vector Xd, across the prediction horizon;
    Computes the system dynamics matrices Aqp and Bqp, across the prediction horizon;

    */
    //current_euler = _G2B_RotMat.eulerAngles(0, 1, 2);
    
    //currentStates << 0.0, 0.0, _yaw, _posBody, _lowState->getGyroGlobal(), _velBody, -g;
    currentStates << _rpy, _posBody, _lowState->getGyroGlobal(), _velBody, -g;
    //ROS_INFO("Speed in x: %.2f s", _velBody(0));
    //std::cout << "_pcd" << std::endl 
       //       << _pcd << std::endl;
    //std::cout << "_posBody" << std::endl 
              //<< _posBody << std::endl;

    // Desired States
    for (int i = 0; i < (mpc_N - 1); i++)
        Xd.block<nx, 1>(nx * i, 0) = Xd.block<nx, 1>(nx * (i + 1), 0);
        Xd(nx * (mpc_N - 1) + 2) = _yawCmd;
    for (int j = 0; j < 3; j++)
        Xd(nx * (mpc_N - 1) + 3 + j) = _pcd(j);
    for (int j = 0; j < 3; j++)
        Xd(nx * (mpc_N - 1) + 6 + j) = _wCmdGlobal(j);
    for (int j = 0; j < 3; j++)
        Xd(nx * (mpc_N - 1) + 9 + j) = _vCmdGlobal(j);

    // 单刚体动力学假设下的 Ac 和 Bc 矩阵
    // Ac
    Ac.setZero();
    R_curz = Rz3(_yaw);
    Ac.block<3, 3>(0, 6) = R_curz.transpose();
    Ac.block<3, 3>(3, 9) = I3;
    Ac(11, nu) = 1;
    Ac(12, nu) = 1; 

    // Bc
    Mat3 Ic_W_inv;
    Ic_W_inv = (R_curz * Ic * R_curz.transpose()).inverse(); // Inverse of A1 inertia in world coordinates
    Bc.setZero();
    for (int i = 0; i < 4; i++){
        Bc.block<3, 3>(6, 3 * i) =
                Ic_W_inv * CrossProduct_A(_posFeet2BGlobal.block<3, 1>(0, i));
        Bc.block<3, 3>(9, 3 * i) =
                (1 / _mass) * I3;
    }

    // Ad = I + Ac * dt，Bd = Bc * dt
    Ad.setZero();
    Bd.setZero();

    Ad = Eigen::Matrix<double, nx, nx>::Identity() + Ac * d_time;
    Bd = Bc * d_time;

    // Aqp = [  A,
    //         A^2,
    //         A^3,
    //         ...
    //         A^k]' 
    // with a size of (nx * mpc_N, nx)

    // Bqp = [A^0*B,        0,       0,   ...       0 
    //         A^1*B,       B,            ...
    //         A^2*B,     A*B,       B,   ...       0
    //         ...
    //         A^(k-1)*B, A^(k-2)*B, A^(k-3)*B, ... B]  
    // with a size of (nx * mpc_N, nu * mpc_N)
    
    Bqp.resize(nx * mpc_N, nu * mpc_N);
    Aqp.setZero();
    Bqp.setZero();
    Bd_list.setZero();

    for (int i = 0; i < mpc_N; ++i) {
        if (i == 0) {
            Aqp.block<nx, nx>(nx * i, 0) = Ad;
        } else {
            Aqp.block<nx, nx>(nx * i, 0) = 
                Aqp.block<nx, nx>(nx * (i-1), 0) * Ad;
        }
    
        for (int j = 0; j <= i; ++j) {
            if (i == j) {
                Bqp.block<nx, nu>(nx * i, nu * j) = Bd;
            } else {
                // 计算 A_d^{i-j_F-1} * Bd
                Eigen::MatrixXd Ad_power = Eigen::MatrixXd::Identity(nx, nx);
                for (int k = 0; k < i-j-1; ++k) {
                    Ad_power *= Ad;
                }
                Bqp.block<nx, nu>(nx * i, nu * j) = Ad_power * Bd;
            }
        }
    }

    dense_hessian.resize(nu * mpc_N, nu * mpc_N);
    dense_hessian.setZero();
    dense_hessian = (Bqp.transpose() * Q * Bqp); 
    dense_hessian += R;
    dense_hessian = dense_hessian * 2;

    gradient.setZero();
    
    Eigen::Matrix<double, nx * mpc_N, 1> error = Aqp * currentStates;
    error -= Xd;
    gradient = 2 * error.transpose() * Q * Bqp;

}

void State_MPC::Publishing() // States Publishing: rosbag record -a  ---> plotjuggler.
{
    msg_euler.x = currentStates(0); // Roll
    msg_euler.y = currentStates(1); // Pitch
    msg_euler.z = currentStates(2); // Yaw
    pub_euler.publish(msg_euler);

    msg_pos.x = currentStates(3); // x
    msg_pos.y = currentStates(4); // y
    msg_pos.z = currentStates(5); // z
    pub_pos.publish(msg_pos);

    msg_speed.x  = currentStates(9); // vx
    msg_speed.y = currentStates(10); // vy
    msg_speed.z = currentStates(11); // vz
    pub_speed.publish(msg_speed);

    cmd_euler.x = _dYaw;
    cmd_euler.y = _dYawCmd;
    cmd_euler.z = _yawCmd;
    pubcmd_euler.publish(cmd_euler);

    cmd_pos.x = _pcd(0);
    cmd_pos.y = _pcd(1);
    cmd_pos.z = _pcd(2);
    pubcmd_pos.publish(cmd_pos);
  
    cmd_speed.x = _vCmdGlobal(0);
    cmd_speed.y = _vCmdGlobal(1);
    cmd_speed.z = _vCmdGlobal(2);
    pubcmd_speed.publish(cmd_speed);
}

void State_MPC::ConstraintsSetup() // Setting up the constraint matrices for the QP solver.
{
    int contactLegNum = 0;
    int swingLegNum = 0;

    for (int i = 0; i < 4; ++i)
    {
        if ((*_contact)(i) == 1)
        {
            contactLegNum += 1;   
        } else {
            swingLegNum += 1;
        }
    }

    CI_.resize(5 * contactLegNum * mpc_N, nu * mpc_N); // CI^T
    CE_.resize(3 * swingLegNum * mpc_N, nu * mpc_N);   // CE^T
    ci0_.resize(5 * contactLegNum * mpc_N, 1);
    ce0_.resize(3 * swingLegNum * mpc_N, 1);

    CI_.setZero();
    ci0_.setZero();
    CE_.setZero();
    ce0_.setZero();

    for (int k = 0; k < mpc_N; k++)
    {
        int ceID = 0;
        int ciID = 0;
        for (int i = 0; i < 4; ++i)
        {
            if ((*_contact)(i) == 1)
            {
                CI_.block<5, 3>(5 * contactLegNum * k + 5 * ciID, nu * k + 3 * i) = miuMat;
                ++ciID;
            }
            else
            {
                CE_.block<3, 3>(3 * swingLegNum * k + 3 * ceID, nu * k + 3 * i) = I3;
                ++ceID;
            }
        }
        
        for (int i = 0; i < contactLegNum * mpc_N; ++i) {
          ci0_.segment(i * 5, 5) << 0.0, 0.0, 0.0, 0.0, fz_max;
        }

       // std::cout << "********1 ci0(MPC)********" << std::endl
           //  << ci0_ << std::endl;
        //std::cout << "********2 CI'(MPC)********" << std::endl
         //    << CI_ << std::endl;
        
             
    }
}

void State_MPC::calcFe()
{   
    SetMatrices();
    Publishing();
    ConstraintsSetup();
    solveQP();
    _forceFeetGlobal = vec12ToVec34(F_);

}

void State_MPC::solveQP() // Convert the formulation to the QP solver and output the contact forces.
{
    /* QuadProg++ Formatting：

    min J = 0.5 * x' G x + g0' x
    subject to : CE^T x + ce0 = 0 
                 CI^T x + ci0 >= 0 

    Dimensions are as follows:              n = nu * mpc_N                   
        G: n * n                            m = 3 * swingLegNum * mpc_N
		g0: n * 1                           p = 5 * contactLegNum * mpc_N    
                            
		CE: n * p
	    ce0: p * 1
				
	    CI: n * m
        ci0: m * 1

        x: n * 1
    */

    int n = nu * mpc_N;
    int m = ce0_.size(); // 3 * swingLegNum * mpc_N
    int p = ci0_.size(); // 5 * contactLegNum * mpc_N

    G.resize(n, n);
    CE.resize(n, m);
    CI.resize(n, p);
    g0.resize(n);
    ce0.resize(m);
    ci0.resize(p);
    x.resize(n);
    x_vec.resize(n,1);

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(dense_hessian);
    if (eigensolver.eigenvalues().minCoeff() <= 0) {
        ROS_INFO("Hessian is not <POSITIVE DEFINITE>");
    } // Hessian矩阵正定性判断

    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            G[i][j] = dense_hessian(i, j);
        }
    }

    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < m; ++j)
        {
            CE[i][j] = (CE_.transpose())(i, j);
        }
    }

    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < p; ++j)
        {
            CI[i][j] = (CI_.transpose())(i, j);
        }
    }

    for (int i = 0; i < n; ++i)
    {
        g0[i] = gradient[i];
    }

    for (int i = 0; i < m; ++i)
    {
        ce0[i] = ce0_[i];
    }

    for (int i = 0; i < p; ++i)
    {
        ci0[i] = ci0_[i];
    }

    // std::cout << "n:" << n << "m:" << m << "p:" << p << std::endl;

    double JJJ = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);

    for (int i = 0; i < n; ++i)
    {
        x_vec(i) = x[i];
    }

    //double cost_value = (0.5 * x_vec.transpose() * dense_hessian * x_vec + gradient.transpose() * x_vec).value();
    // J = X^T Q X + U^T R U 
    //prediction_X = Aqp * currentStates + Bqp * x_vec - Xd;    
    //double cost_func = (prediction_X.transpose() * Q * prediction_X + x_vec.transpose() * R * x_vec).value();
    //std::cout << "cost:" << cost_func << std::endl;
    
    for (int i = 0; i < 12; ++i)
    {
        F_[i] = -x[i];  
    }
    
}

Eigen::Matrix<double, 3, 3> State_MPC::CrossProduct_A(Eigen::Matrix<double, 3, 1> A)
{
    Eigen::Matrix<double, 3, 3> M;
    M << 0.0, -A[2], A[1],
        A[2], 0.0, -A[0],
        -A[1], A[0], 0.0;
    return M;
}

Eigen::Matrix<double, 3, 3> State_MPC::Rz3(double theta)
{  // local to world
    // for 2D-XY vector, rotation matrix along z axis
    Eigen::Matrix<double, 3, 3> M;
    M << cos(theta), -sin(theta), 0,
        sin(theta), cos(theta), 0,
        0, 0, 1;
    return M;
}