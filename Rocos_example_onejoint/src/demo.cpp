

#include <csignal>
#include <cstdio>
#include <cstdlib>

#include <rocos_app/drive.h>
#include <rocos_app/ethercat/hardware.h>
#include <rocos_app/ethercat/hardware_sim.h>
#include <fstream>
#include <iostream>
#include <rocos_app/robot.h>
#include <rocos_app/robot_service.h>
#include <string>
#include <gflags/gflags.h>

DEFINE_string(urdf, "config/robot_onejoint.urdf", "Urdf file path");
DEFINE_string(base, "base_link", "Base link name");
DEFINE_string(tip, "link_1", "Tip link name");

bool isRuning = true;

#pragma region //*测试9  完整上电保护程序
Eigen::Matrix3d KDLRotationToEigenMatrix(const KDL::Rotation &rotation)
{
    Eigen::Matrix3d matrix;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            matrix(i, j) = rotation(i, j);
        }
    }
    return matrix;
}
namespace rocos
{
    /**
     * @brief 字符串切割函数
     *
     * @param str 待切割字符串
     * @param tokens 结果存储
     * @param delim 切割符
     */
    boost::shared_ptr<HardwareInterface> hw = boost::make_shared<Hardware>(); // 真实机械臂

    Robot robot(hw, FLAGS_urdf, FLAGS_base, FLAGS_tip);
    void moveLWithVector(const std::vector<double> &pose, double speed = 1.05, double acceleration = 1.4,
                         double time = 0.0, double radius = 0.0, bool asynchronous = false, int max_running_count = 10)
    {
        if (pose.size() != 6)
        {
            // 处理错误，输入向量的大小必须为6
            std::cerr << "Error: Input vector must have 6 elements." << std::endl;
            return;
        }

        // 解析输入向量的前三个元素作为平移部分，后三个元素作为RPY旋转部分
        KDL::Vector translation(pose[0], pose[1], pose[2]);
        KDL::Rotation rotation = KDL::Rotation::RPY(pose[3], pose[4], pose[5]);

        // 创建KDL::Frame对象
        KDL::Frame frame(rotation, translation);
        robot.MoveL(frame, speed, acceleration, time, radius, asynchronous, max_running_count);
    }
    void moveJWithVector(const std::vector<double> &pose,
                         double speed = 1.05,
                         double acceleration = 1.4,
                         double time = 0.0,
                         double radius = 0.0,
                         bool asynchronous = false)
    {
        if (pose.size() != 7)
        {
            // 处理错误，输入向量的大小必须为6
            std::cerr << "Error: Input vector must have 7 elements." << std::endl;
            return;
        }

        // 创建一个KDL::JntArray对象，并从输入向量中复制值
        KDL::JntArray q(pose.size());
        for (size_t i = 0; i < pose.size(); ++i)
        {
            q(i) = pose[i];
        }

        // 调用原始的MoveJ函数，传递KDL::JntArray对象
        robot.MoveJ(q, speed, acceleration, time, radius, asynchronous);
    }

    void Robot::test()
    {

        //**变量初始化 **//
        std::string str{""};
        std::ifstream csv_null_motion;

        char tem[2048];
        std::vector<std::string> tokens;
        std::vector<KDL::JntArray> servo_data;
        KDL::JntArray joints(_joint_num);
        KDL::JntArray last_joints(_joint_num);

        int row_index = 1;

        JC_helper::TCP_server my_server;

        my_server.init();
        boost::thread(&JC_helper::TCP_server::RunServer, &my_server).detach(); // 开启服务器

        //**-------------------------------**//

#pragma region //*电机使能检查

        for (int i{0}; i < jnt_num_; i++)
        {
            if (joints_[i]->getDriveState() != DriveState::OperationEnabled)
            {
                for (int j{0}; j < 1; j++)
                {
                    PLOG_ERROR << "电机[" << i << "] 未使能，确定主站已初始化完成了？,输入y确认";
                    std::cin >> str;
                    if (str != std::string_view{"y"})
                    {
                        PLOG_ERROR << "未输入y, 判断主站 {未} 初始化完成,程序关闭";

                        exit(0);
                    }
                }
            }
        }

        setEnabled();
#pragma endregion

        while (isRuning)
        {
            str.clear();

            PLOG_INFO << "当前环境是否安全,如果是,输入run开始执行程序";
            std::cin >> str;

            if (str == std::string_view{"run"})
            {
                using namespace KDL;
                KDL::JntArray q_target(_joint_num);
                q_target(0) = -45 * M_PI / 180;
                q_target(1) = 45 * M_PI / 180;
                q_target(2) = 0 * M_PI / 180;
                q_target(3) = 90 * M_PI / 180;
                q_target(4) = 0 * M_PI / 180;
                q_target(5) = -45 * M_PI / 180;
                q_target(6) = 0 * M_PI / 180;

                KDL::JntArray to_home(_joint_num);
                to_home(0) = 0 * M_PI / 180;
                to_home(1) = 0 * M_PI / 180;
                to_home(2) = 0 * M_PI / 180;
                to_home(3) = 0 * M_PI / 180;
                to_home(4) = 0 * M_PI / 180;
                to_home(5) = 0 * M_PI / 180;
                to_home(6) = 0 * M_PI / 180;
                // std::vector<double> joint = {0.000275 ,0.797012 ,-0.000019 ,1.571365 ,0.000083 ,1.221532 ,-0.071660};
                // moveJWithVector(joint,0.1,0.1,0,0,false);
                // std::vector<double> pose = {0.473642, 0.000139, 0.140273,3.107120, -0.447079, -3.061883};
                // moveLWithVector(pose);
                // 
                KDL::Frame p0;
                kinematics_.JntToCart( q_target, p0 );
                KDL::Rotation rotation = p0.M;

                // 将 KDL::Rotation 转换为 Eigen::Matrix3d
                Eigen::Matrix3d rotation_matrix = KDLRotationToEigenMatrix(rotation);

                // 输出 Eigen::Matrix3d
                std::cout << "Rotation Matrix:" << std::endl;
                std::cout << rotation_matrix << std::endl;

                //**---------------------------------------------------------------------------------------------------------------**//
                // //** movej **//
                // MoveJ( q_target, 0.2, 0.2, 0, 0, false );
                // MoveJ( to_home, 0.2, 0.2, 0, 0, false );

                // //**---------------------------------------------------------------------------------------------------------------**//
                // //** 正方形**//
                // //将机械臂移动到合适位置
                // MoveJ( q_target, 0.2, 0.2, 0, 0, false );

                // KDL::Frame f_p0;
                // KDL::Frame f_p1;
                // KDL::Frame f_p2;
                // KDL::Frame f_p3;
                // KDL::Frame f_p4;
                // KDL::Frame f_p5;
                // KDL::Frame f_p6;

                // for ( int i = 0; i < 7; i++ )
                //     joints_[ i ]->setMode( ModeOfOperation::CyclicSynchronousPositionMode );

                // kinematics_.JntToCart( q_target, f_p0 );

                // f_p1 = f_p0 * KDL::Frame{ KDL::Vector{ 0.0, -0.15, 0.0 } };
                //  MoveL( f_p1, 0.07, 0.2, 0, 0, false );
                // f_p2 = f_p1 * KDL::Frame{ KDL::Vector{ -0.15, 0.0, 0.0 } };
                //  MoveL( f_p2, 0.07, 0.2, 0, 0, false );
                // f_p3 = f_p2 * KDL::Frame{ KDL::Vector{ 0.0, 0.15, 0.0 } };
                //  MoveL( f_p3, 0.07, 0.2, 0, 0, false );
                // f_p4 = f_p3 * KDL::Frame{ KDL::Vector{ 0.15, 0, 0.0 } };
                //  MoveL( f_p4, 0.07, 0.2, 0, 0, false );
                // Home
                // MoveJ( to_home, 0.2, 0.2, 0, 0, false );

                // //**--------------------------------------------------------------------------------------------------------------------**//
                // //圆弧路径
                // //将机械臂移动到合适位置
                // MoveJ( q_target, 0.2, 0.2, 0, 0, false );

                // KDL::Frame f_p0;
                // kinematics_.JntToCart( q_target, f_p0 );

                // KDL::Frame f_c_p1 = f_p0 * KDL::Frame{ KDL::Vector{ -0.15, 0.0, 0.0 } };
                // KDL::Frame f_c_p2 = f_p0 * KDL::Frame{ KDL::Vector{ 0.0, 0.15, 0.0 } };
                // MoveC( f_c_p1, f_c_p2, 0.04, 0.3, 0, 0, Robot::OrientationMode::UNCONSTRAINED, false );

                // // //Home
                // MoveJ( to_home, 0.2, 0.2, 0, 0, false );

                ///**--------------------------------------------------------------------------------------------------------------------------**//
                // //打印关节角度和笛卡尔空间位姿
                // MoveJ(q_target, 0.3, 1, 0, 0, false);

                // KDL::Frame f_p0; // 笛卡尔空间位姿存储初始化
                // kinematics_.JntToCart(q_target, f_p0);      //关节空间转笛卡尔空间
                // for (int i = 0; i < 7; i++)
                // {
                //     std::cout << "关节角度" << pos_[i] << std::endl;
                // }
                // std::cout << "关节转末端笛卡尔位置  :\n" << f_p0 << std::endl;
                // // std::cout << "关节转末端笛卡尔位置  :\n" << f_p0.p << std::endl;
                // // std::cout << "关节转末端笛卡尔姿态  :\n" << f_p0.M << std::endl;
                // ////std::cout << "末端笛卡尔位姿  :\n" << flange_ << std::endl;

                //      ///**--------------------------------------------------------------------------------------------------------------------------**//
                //    // 末端位置不变，姿态发生改变
                //     bool pose_rotation = true;
                //     KDL::Frame f_p0;
                //     KDL::Frame f_p1;

                //     kinematics_.JntToCart( q_target, f_p0 );

                //     MoveJ( q_target, 0.2, 0.2, 0, 0, false );
                //     if (pose_rotation)
                //     {

                //         double theta = {0.7}; // rad
                //         // 绕X
                //         f_p1= f_p0 * KDL::Frame{ KDL::Rotation::RPY(theta,0,0) };
                //         // 绕Y
                //         // f_p1= f_p0 * KDL::Frame{ KDL::Rotation::RPY(0,theta,0) };
                //         // 绕Z
                //         // f_p1 = f_p0 * KDL::Frame{KDL::Rotation::RPY(0, 0, theta)};

                //         std::cout << "末端笛卡尔位姿  :\n" << f_p1 << std::endl;
                //         MoveL(f_p1, 0.05, 0.1, 0, 0, false);
                //     }

                //  ///**--------------------------------------------------------------------------------------------------------------------------**//

                // // // 简单的沿末端TCP的Y轴方向移动函数
                // // // 公式：A_M_B*P_B+V
                // // std::cout << "------------------------------------" << std::endl;
                // bool TCPmove = true;
                // KDL::Frame f_p0;
                // KDL::Frame f_p1;

                // kinematics_.JntToCart( q_target, f_p0 );
                // MoveJ( q_target, 0.2, 0.2, 0, 0, false );
                // if (TCPmove)
                // { // 沿 法兰盘坐标系Y轴移动-0.1
                //     // f_p1.p = f_p0.M * KDL::Vector{0.0, -0.1, 0.0} + f_p0.p;

                //     // f_p1 = f_p0 * KDL::Frame{ KDL::Vector{ 0.0, -0.15, 0.0 } };
                // //  MoveL( f_p1, 0.07, 0.2, 0, 0, false );

                //     // 沿 法兰盘坐标系X轴移动-0.1
                //    // f_p1.p = f_p0.M * KDL::Vector{-0.1, 0.0, 0.0} + f_p0.p;
                //     // 沿 法兰盘坐标系Z轴移动-0.1
                //    f_p1.p = f_p0.M * KDL::Vector{0.0, 0.0, -0.1} + f_p0.p;
                //     f_p1.M = f_p0.M;
                //     std::cout << "关节转末端笛卡尔位姿  :\n" << f_p1 << std::endl;
                //     MoveL(f_p1, 0.05, 0.1, 0, 0, false);
                // }

                // else
                // {
                //         PLOG_ERROR << "不安全环境,电机抱闸";
                //         setDisabled( );

                //         return;
                // }
            }

            PLOG_INFO << "全部测试结束,goodbye!";
        }
    }
} // namespace rocos
#pragma endregion

/// \brief 处理终端的Ctrl-C信号
/// \param signo
void signalHandler(int signo)
{
    if (signo == SIGINT)
    {
        std::cout << "\033[1;31m"
                  << "[!!SIGNAL!!]"
                  << "INTERRUPT by CTRL-C"
                  << "\033[0m" << std::endl;

        isRuning = false;
        exit(0);
    }
}

int main(int argc, char *argv[])
{
    if (signal(SIGINT, signalHandler) == SIG_ERR)
    {
        std::cout << "\033[1;31m"
                  << "Can not catch SIGINT"
                  << "\033[0m" << std::endl;
    }

    using namespace rocos;

    gflags::ParseCommandLineFlags(&argc, &argv, true);
    //**-------------------------------**//

    //    boost::shared_ptr< HardwareInterface > hw = boost::make_shared< HardwareSim >( _joint_num );  // 仿真

    auto robotService = RobotServiceImpl::getInstance(&robot);

    //------------------------wait----------------------------------
    //std::thread thread_test{ &rocos::Robot::test, &robot };

    //------------------------wait----------------------------------
    robotService->runServer();
    //thread_test.join( );

    return 0;
}
