

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include "Iir.h"
#include <rocos_app/drive.h>
#include <rocos_app/ethercat/hardware.h>
#include <rocos_app/ethercat/hardware_sim.h>
#include <fstream>
#include <iostream>
#include <rocos_app/robot.h>
#include <rocos_app/robot_service.h>
#include <string>
#include <gflags/gflags.h>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <iostream>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <Eigen/Core>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames_io.hpp>
#include <Eigen/Dense>
using namespace std::chrono;
DEFINE_string(urdf, "config/robot_sun_new.urdf", "Urdf file path");
DEFINE_string(base, "base_link", "Base link name");
DEFINE_string(tip, "link_7", "Tip link name");

KDL::Tree kdl_tree;
KDL::Chain robotChain;
bool isRuning = true;
std::array<Iir::Butterworth::LowPass<2>, 7> filter_array{};
const float samplingrate = 1000; // Hz
const float cutoff_frequency = 3;
#pragma region //*测试9  完整上电保护程序
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

    void signalHandler(int signo)
    {
        if (signo == SIGINT)
        {
            std::cout << "\033[1;31m"
                      << "[!!SIGNAL!!]"
                      << "INTERRUPT by CTRL-C"
                      << "\033[0m" << std::endl;

            isRuning = false;
            robot.setDisabled();
            // outputFile_246.close();
            // outputFile_3.close();
            // outputFile_5.close();
            std::cout << "Data saved to joint_torques.csv" << std::endl;
            exit(0);
        }
    }
    class MyFilter
    {
    public:
        MyFilter(int windowsize) : windowsize_(windowsize), buffer_data_(windowsize), buffer_index_(0), sum_(0), is_filled_(false)
        {
        }

        double filter(const double &x, bool reset = false)
        {
            double mean_value = 0;

            if (reset)
            {
                buffer_data_.assign(windowsize_, 0);
                buffer_index_ = 0;
                sum_ = 0;
                is_filled_ = false;
                return 0;
            }

            buffer_index_ = (buffer_index_ + 1) % windowsize_;

            if (is_filled_)
            {
                sum_ = sum_ - buffer_data_[buffer_index_];
                sum_ = sum_ + x;
                buffer_data_[buffer_index_] = x;
                mean_value = sum_ / windowsize_;
            }
            else
            {
                sum_ = sum_ + x;
                buffer_data_[buffer_index_] = x;
                mean_value = sum_ / (buffer_index_ + 1);
            }

            if (!is_filled_ && buffer_index_ == windowsize_ - 1)
                is_filled_ = true;

            return mean_value;
        }

    private:
        int windowsize_;
        std::vector<double> buffer_data_;
        int buffer_index_;
        double sum_;
        bool is_filled_;
    };

    double slidingAverageFilter(std::vector<double> &buffer, double newValue)
    {
        buffer.push_back(newValue);
        if (buffer.size() > buffer.capacity())
        {
            buffer.erase(buffer.begin());
        }

        double sum = 0.0;
        for (double value : buffer)
        {
            sum += value;
        }

        return sum / buffer.size();
    }
    double segmentedSigmoid(double x, double scale = 5, double shift = 23.0, double amplitude = 30.0)
    {
        // 46->5,23,30
        // 5->6.5,26,30
        double y;
        if (x >= 0)
        {
            y = amplitude / (1 + 2 * exp(-(x - shift) / scale)); // 正区间
        }
        else
        {
            y = -amplitude / (1 + 2 * exp(-(-x - shift) / scale)); // 负区间
        }
        return y;
    }

    void Robot::test()
    {

        int _joint_num = 7;                                  /* Your joint number */
        if (!kdl_parser::treeFromFile(FLAGS_urdf, kdl_tree)) // 建立tree
        {
            // 加载URDF文件失败，处理错误
            std::cout << "加载URDF文件失败" << std::endl;
            exit(0);
        }
        if (!kdl_tree.getChain(FLAGS_base, FLAGS_tip, robotChain)) // 建立运动链
        {
            // 获取机器人链失败，处理错误
            std::cout << "获取机器人链失败" << std::endl;
            exit(0);
        }

#pragma region // RNE
        KDL::JntArray joints(_joint_num);
        KDL::JntArray last_joints(_joint_num);
        KDL::JntArray joint_velocities(_joint_num);
        KDL::JntArray joint_accelerations(_joint_num);
        KDL::JntArray joint_real_torques(_joint_num);
        KDL::JntArray joint_cmd_torques(_joint_num);
        KDL::JntArray q_target1(_joint_num);
        KDL::JntArray q_target2(_joint_num);
        KDL::JntArray q_target3(_joint_num);
        KDL::JntArray q_target4(_joint_num);
        KDL::JntArray q_target5(_joint_num);
        KDL::JntArray q_target_mid(_joint_num);
        // Define external forces (Wrenches) acting on each segment (link)
        KDL::Wrenches external_forces(robotChain.getNrOfSegments());
        std::cout << "the num of robot is :" << robotChain.getNrOfSegments() << std::endl;
        // Set external forces (e.g., due to end-effector contact or other external factors)
        // Define gravity vector
        KDL::Vector gravity(0.0, 0.0, 9.81); // Assuming gravity in the negative Z direction
        // Create the ChainIdSolver_RNE object
        KDL::ChainIdSolver_RNE rne_solver(robotChain, gravity);
        // Create a KDL JntArray to store computed joint torques
        KDL::JntArray joint_torques(_joint_num);
        int row_index = 1;
        JC_helper::TCP_server my_server;
        my_server.init();
        boost::thread(&JC_helper::TCP_server::RunServer, &my_server).detach(); // 开启服务器
#pragma endregion

#pragma region //*电机使能检查

        setEnabled();
#pragma endregion

#pragma region //*滤波器

        //** 移动均值滤波器初始化 **//
        const size_t filterSize = 10; // 滤波器窗口大小
        std::vector<MyFilter> filters(_joint_num, MyFilter(filterSize));
        //**-------------------------------**//
        //** 巴低通滤波器初始化 **//
        for (int i = 0; i < _joint_num; i++)
        {
            filter_array[i].setup(2, samplingrate, cutoff_frequency);
            q_target1(i) = getJointPosition(i);
        }
#pragma endregion
#pragma region //
        int com_joint[3] = {1, 3, 5};
        int com_num = 3;
        double joint_tor[com_num];
        double d_fext = 0.0;
        double fext[com_num] = {0.0, 0.0, 0.0};
        double pos_prev[_joint_num] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double fext_prev = 0.0;
        double com_fext[com_num] = {0.0, 0.0, 0.0};
        double pose[com_num] = {getJointPosition(1), getJointPosition(3), getJointPosition(5)};
        double a_joint[com_num] = {140.6, 90.19, 42.72};
        double b_joint[com_num] = {-6.316, -1.743, -0.8113};
        double a_6 = 42.72;
        double b_6 = -0.8113;
        double a_4 = 90.19;
        double b_4 = -1.743;
        double a_5 = 85.41;
        double b_5 = -2.592;
        double a_2 = 140.6;
        double b_2 = -6.316;
        double K[com_num] = {0.01, 0.01, 0.01};
        double M[com_num] = {20, 15, 8};
        double B[com_num] = {2 * sqrt(M[0] * K[0]) * 30, 2 * sqrt(M[1] * K[1]) * 20, 2 * sqrt(M[2] * K[2]) * 10};
        // double K = 0.01;
        // double M = 8.0;
        // double B = 2 * sqrt(M * K) * 16; //
        double acc[com_num] = {0.0, 0.0, 0.0};
        double vel[com_num] = {0.0, 0.0, 0.0};
        double dt = 0.001;

        int j = 0;
        double last_pose[com_num] = {getJointPosition(1), getJointPosition(3), getJointPosition(5)};
        std::cout << pose[0] << "\t" << pose[1] << "\t" << pose[2] << std::endl;
        //0.791457        0.756879        -1.77775
        while (isRuning)
        {
            for (int i = 0; i < _joint_num; ++i)
            {
                // Set initial joint positions
                joints(i) = getJointPosition(i);
                // std::cout << "pos: " << joints(i) << "\t";
                joint_velocities(i) = (joints(i) - pos_prev[i]) / 0.001; // Set initial joint velocities
                // std::cout<<"joint_velocities"<<(joints(i) - pos_prev[i]) / 0.001;
                joint_accelerations(i) = 0; // Set initial joint accelerations
                pos_prev[i] = joints(i);
            }

            rne_solver.CartToJnt(joints, joint_velocities, joint_accelerations, external_forces, joint_torques);

            for (int i = 0; i < _joint_num; ++i)
            {

                // joint_tor = averageFilter_JointTor(i, Num_averge) / 1000 / Sensitivity[i];
                // joint_tor[i]=((getJointLoadTorque(i)/ 1000.0 )-ZeroOffset[i])/ Sensitivity[i];
                joint_tor[i] = filter_array[i].filter(getJointLoadTorque(i)) * 0.001;

                // std::cout << "load: " << joint_tor[i] << "\t";
                //  joint_tor[i] = filters[i].filter(((getJointLoadTorque(i) / 1000.0) - ZeroOffset[i]) / Sensitivity[i]);
            }
            // outputFile << "1";

            if (j > 1000)
            {
                for (int i = 0; i < com_num; ++i)
                {

                    fext[i] = joint_tor[com_joint[i]] * a_joint[i] + b_joint[i] - joint_torques(com_joint[i]);
                    com_fext[i] = segmentedSigmoid(fext[i]);
                    //  std::cout << "joint_tor: " << joint_tor[com_joint[i]]*a_joint[i]+b_joint[i];
                    // std::cout << "theory: " << joint_torques(com_joint[i]);
                    // std::cout << "com_fext: " << com_fext[i] <<",";

                    acc[i] = (com_fext[i] - B[i] * vel[i]) / M[i];
                    //     // acc+=jerk*dt;
                    vel[i] = vel[i] + acc[i] * dt;
                    pose[i]= pose[i]+vel[i]*dt;
                    //std::cout << acc[i] << "\t" << vel[i] << "\t" << pose[i] << std::endl;
                   // std::cout <<  pose[i];
                    if ((abs(pose[i] - last_pose[i])) < 0.00004)
                    {
                        pose[i] = last_pose[i];
                    }
                    q_target1(com_joint[i]) = pose[i];
                    if (pose[i] < -90.0 * M_PI / 180.0 || pose[i] > 110.0 * M_PI / 180.0)
                    {
                        std::cerr << "Joint angle out of bounds" << com_joint[i] << " " << pose[i] << std::endl;
                        std::exit(1);
                    }

                    last_pose[i] = pose[i];
                    // }
                }
                //std::cout<<std::endl;
                servoJ(q_target1);
                
            }

            j++;
        }
    }

#pragma endregion
    // PLOG_INFO << "全部测试结束,goodbye!";
}
// namespace rocos
#pragma endregion

/// \brief 处理终端的Ctrl-C信号
/// \param signo

int main(int argc, char *argv[])
{

    if (signal(SIGINT, rocos::signalHandler) == SIG_ERR)
    {
        std::cout << "\033[1;31m"
                  << "Can not catch SIGINT"
                  << "\033[0m" << std::endl;
    }

    using namespace rocos;

    gflags::ParseCommandLineFlags(&argc, &argv, true);
    //**-------------------------------**//

    // boost::shared_ptr<HardwareInterface> hw = boost::make_shared<HardwareSim>(_joint_num); // 仿真

    auto robotService = RobotServiceImpl::getInstance(&robot);

    //------------------------wait----------------------------------
    std::thread thread_test{&rocos::Robot::test, &robot};

    //------------------------wait----------------------------------
    robotService->runServer();

    thread_test.join();

    return 0;
}
