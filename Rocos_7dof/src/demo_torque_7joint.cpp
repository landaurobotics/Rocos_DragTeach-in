

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
std::ofstream outputFile_246("/home/landau/Rocos_7dof/src/joint_torques_246joint.csv");
std::ofstream outputFile_3("/home/landau/Rocos_7dof/src/joint_torques_3joint.csv");
std::ofstream outputFile_5("/home/landau/Rocos_7dof/src/joint_torques_5joint.csv");
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
            outputFile_246.close();
            outputFile_3.close();
            outputFile_5.close();
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
            q_target1(i)=0.0;
        }
#pragma endregion
#pragma region // Open a file for writing
        double joint_tor[_joint_num];
      

        if (outputFile_246.is_open())
        {
            for (double angle = 90.0; angle >= -90.0; angle -= 0.5)
            {
                q_target1(1) = angle / 180.0 * M_PI;
                MoveJ(q_target1, 0.1, 0.1);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                for (int i = 0; i < _joint_num; ++i)
                {
                    // Set initial joint positions
                    joints(i) = getJointPosition(i);
                    joint_velocities(i) = 0;    // Set initial joint velocities
                    joint_accelerations(i) = 0; // Set initial joint accelerations
                    std::cout<<q_target1(i)<<",";
                }
                std::cout<<std::endl;
                rne_solver.CartToJnt(joints, joint_velocities, joint_accelerations, external_forces, joint_torques);
                for (int j = 0; j < 2001; j++)
                {
                   
                        joint_tor[1] = filter_array[1].filter((getJointLoadTorque(1))); // 电压
                        joint_tor[3] = filter_array[3].filter((getJointLoadTorque(3))); // 电压
                        joint_tor[5] = filter_array[5].filter((getJointLoadTorque(5))); // 电压
                    

                    if (j > 1000)
                    {
                        outputFile_246 << joint_tor[1] << "," << joint_tor[3] << "," << joint_tor[5] << "," << joint_torques(1) << "," << joint_torques(3)<< "," << joint_torques(5)<< std::endl;
                    }
                    robot.hw_interface_->waitForSignal(0);
                }
            }

            // Close the file
            outputFile_246.close();
            std::cout << "Data saved to joint_torques.csv" << std::endl;
        }
        else
        {
            std::cout << "Unable to open the file." << std::endl;
        }

        if (outputFile_3.is_open())
        {
            

            for (double angle = 90.0; angle >= -90.0; angle -= 0.5)
            {
                //q_target1(1) = -90 / 180.0 * M_PI;
                q_target1(3) = 90.0 / 180.0 * M_PI;
                q_target1(2) = angle / 180.0 * M_PI;
                MoveJ(q_target1, 0.1, 0.1);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                for (int i = 0; i < _joint_num; ++i)
                {
                    // Set initial joint positions
                    joints(i) = getJointPosition(i);
                    joint_velocities(i) = 0;    // Set initial joint velocities
                    joint_accelerations(i) = 0; // Set initial joint accelerations
                }
                rne_solver.CartToJnt(joints, joint_velocities, joint_accelerations, external_forces, joint_torques);
                for (int j = 0; j < 2001; j++)
                {
                  

                        joint_tor[2] = filter_array[2].filter((getJointLoadTorque(2))); // 电压
                    

                    if (j > 1000)
                    {
                        outputFile_3 << joint_tor[2] << "," << joint_torques(2) << std::endl;
                    }
                    robot.hw_interface_->waitForSignal(0);
                }
            }

            // Close the file
            outputFile_3.close();
            std::cout << "Data saved to joint_torques.csv" << std::endl;
        }
        else
        {
            std::cout << "Unable to open the file." << std::endl;
        }
    
     if (outputFile_5.is_open())
        {
            

            for (double angle = 90.0; angle >= -90.0; angle -= 0.5)
            {
                q_target1(5) = 90.0 / 180.0 * M_PI;
                q_target1(4) = angle / 180.0 * M_PI;
                MoveJ(q_target1, 0.1, 0.1);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                for (int i = 0; i < _joint_num; ++i)
                {
                    // Set initial joint positions
                    joints(i) = getJointPosition(i);
                    joint_velocities(i) = 0;    // Set initial joint velocities
                    joint_accelerations(i) = 0; // Set initial joint accelerations
                }
                rne_solver.CartToJnt(joints, joint_velocities, joint_accelerations, external_forces, joint_torques);
                for (int j = 0; j < 2001; j++)
                {
                    

                        joint_tor[4] = filter_array[4].filter((getJointLoadTorque(4))); // 电压
                    
                    if (j > 1000)
                    {
                        outputFile_5 << joint_tor[4] << "," << joint_torques(4) << std::endl;
                    }
                    robot.hw_interface_->waitForSignal(0);
                }
            }

            // Close the file
            outputFile_5.close();
            std::cout << "Data saved to joint_torques.csv" << std::endl;
        }
        else
        {
            std::cout << "Unable to open the file." << std::endl;
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
