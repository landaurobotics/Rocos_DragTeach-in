// Copyright 2021, Yang Luo"
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// @Author
// Yang Luo, PHD
// Shenyang Institute of Automation, Chinese Academy of Sciences.
// email: luoyang@sia.cn

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
using namespace std::chrono;
DEFINE_string(urdf, "config/robot.urdf", "Urdf file path");
DEFINE_string(base, "base_link", "Base link name");
DEFINE_string(tip, "link_7", "Tip link name");
std::ofstream outputFile("/home/landau/Rocos_example/src/joint_torques_fix.csv");
std::ofstream outputFile_ser("/home/landau/Rocos_example/src/joint_target_servo.csv");
bool isRuning = true;
KDL::Tree kdl_tree;
KDL::Chain robotChain;
int joint_num = 5;

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
    double averageFilter_JointTor(int &id, int &num)
    {
        int time_torque = 0;
        double torque = 0.0;
        while (time_torque < num)
        {
            torque += robot.getJointLoadTorque(id);
            // hw->waitForSignal( 0 );
            time_torque++;
        }

        return torque / num;
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
    class JointControl
    {
    public:
        // JointControl(const std::vector<double> &joint_mass, const std::vector<double> &joint_damping,
        //              const std::vector<double> &joint_stiffness, const std::vector<double> &max_velocity,
        //              const std::vector<double> &max_acceleration, const std::vector<double> &init_pose)
        //     : joint_mass_(joint_mass), joint_damping_(joint_damping), joint_stiffness_(joint_stiffness),
        //       max_velocity_(max_velocity), max_acceleration_(max_acceleration), current_position_(init_pose),
        //       current_velocity_(std::vector<double>(7, 0.0)) {

        //       }
        JointControl(const std::vector<double> &joint_mass, const std::vector<double> &joint_damping,
                     const std::vector<double> &joint_stiffness, const std::vector<double> &max_velocity,
                     const std::vector<double> &max_acceleration, const std::vector<double> &init_pose)
        {
            for (int i = 0; i < _joint_num; i++)
            {
                joint_mass_.push_back(joint_mass[i]);
                joint_damping_.push_back(joint_damping[i]);
                joint_stiffness_.push_back(joint_stiffness[i]);
                max_velocity_.push_back(max_velocity[i]);
                max_acceleration_.push_back(max_acceleration[i]);
                current_position_.push_back(init_pose[i]);
                current_velocity_.push_back(0);
            }
        }

        void updateVelocity(double velocity, int num)
        {
            //
            current_velocity_[num] = velocity;
           
        }

        std::vector<double> update(const std::vector<double> &external_torque, double dt)
        {
            for (int i = joint_num; i < 7; ++i)
            {
                double current_acceleration =
                    (external_torque[i] - joint_damping_[i] * current_velocity_[i]) / joint_mass_[i];
                current_acceleration = std::max(-max_acceleration_[i], std::min(max_acceleration_[i], current_acceleration));

                current_velocity_[i] += current_acceleration * dt;
                current_velocity_[i] = std::max(-max_velocity_[i], std::min(max_velocity_[i], current_velocity_[i]));
                
                current_position_[i] += current_velocity_[i] * dt;

                if (current_position_[i] < -90.0 * M_PI / 180.0 || current_position_[i] > 70.0 * M_PI / 180.0)
                {
                    std::cerr << "Joint angle out of bounds" << std::endl;
                    std::exit(1);
                }
            }

            return current_position_;
        }

    private:
        std::vector<double> joint_mass_;
        std::vector<double> joint_damping_;
        std::vector<double> joint_stiffness_;
        std::vector<double> max_velocity_;
        std::vector<double> max_acceleration_;
        std::vector<double> current_position_;
        std::vector<double> current_velocity_;
    };
    void signalHandler(int signo)
    {
        if (signo == SIGINT)
        {
            std::cout << "\033[1;31m"
                      << "[!!SIGNAL!!]"
                      << "INTERRUPT by CTRL-C"
                      << "\033[0m" << std::endl;

            isRuning = false;
            outputFile.close();
            outputFile_ser.close();
            robot.setDisabled();

            exit(0);
        }
    }

    void Robot::test()
    {

#pragma region //*电机使能检查

        setEnabled();
#pragma endregion
        std::vector<double> joint_dec = {10, 20, 6, 3, 2, 1.5, 1.5};
        std::vector<double> joint_mass = {10, 20, 6, 3, 2, 2, 0.5};
        std::vector<double> joint_damping = {2, 15, 10, 5, 1.5, 0, 1};
        std::vector<double> joint_stiffness = {10.0, 10, 10, 10, 10, 10, 10};
        std::vector<double> max_velocity = {0.1, 0.1, 0.1, 0.2, 0.2, 0.2, 0.2};
        std::vector<double> max_acceleration = {0.1, 0.1, 0.1, 0.2, 0.2, 0.8, 0.2};
        std::vector<double> init_pose;
        KDL::JntArray q_target(_joint_num);
        KDL::JntArray q_init(_joint_num);
        //** 滤波器初始化 **//
        const size_t filterSize = 2000; // 滤波器窗口大小
        std::vector<MyFilter> filters(_joint_num, MyFilter(filterSize));
        //**-------------------------------**//
        // 逆动力学计算的准备工作
        std::string urdf_file = "/home/landau/Rocos_example/config/robot_sun.urdf";
        if (!kdl_parser::treeFromFile(urdf_file, kdl_tree)) // 建立tree
        {
            // 加载URDF文件失败，处理错误
            std::cout << "加载URDF文件失败" << std::endl;
            exit(0);
        }
        std::string base_link = "base_link";                     // 设置基准链接
        std::string end_link = "link_7";                         // 设置终端链接
        if (!kdl_tree.getChain(base_link, end_link, robotChain)) // 建立运动链
        {
            // 获取机器人链失败，处理错误
            std::cout << "获取机器人链失败" << std::endl;
            exit(0);
        }
        KDL::JntArray joints(_joint_num);
        KDL::JntArray last_joints(_joint_num);
        KDL::JntArray joint_velocities(_joint_num);
        KDL::JntArray joint_accelerations(_joint_num);
        // Initialize joint positions, velocities, and accelerations (e.g., for a simple joint movement)
        // Define external forces (Wrenches) acting on each segment (link)
        KDL::Wrenches external_forces(robotChain.getNrOfSegments());
        // Set external forces (e.g., due to end-effector contact or other external factors)
        // Define gravity vector
        KDL::Vector gravity(0.0, 0.0, -9.81); // Assuming gravity in the negative Z direction
        // Create the ChainIdSolver_RNE object
        KDL::JntArray joint_torques_rne(_joint_num);
        KDL::ChainIdSolver_RNE rne_solver(robotChain, gravity);
        //
        // 个人控制类的更新准备
        for (int i = 0; i < 7; i++)
        {
            init_pose.push_back(getJointPosition(i));
            // q_init(i)=getJointPosition(i);
        }
        double dt_ctr = 0.001;

        std::vector<double> joint_torques = {0, 0, 0, 0, 0, 0, 0};
        double Sensitivity[_joint_num] = {0.0083702, 0.0085824, 0.013957, 0.013993, 0.013642, 0.030088, 0.032380};
        JointControl joint_ctrl(joint_mass, joint_damping, joint_stiffness, max_velocity, max_acceleration, init_pose);
        int fif_size = 100;
        double ZeroOffset[_joint_num] = {5.32305, 4.12746, 1.99957 - 0.3, 1.23344, 1.72533, 0.375681, 1.23481};
        std::vector<double> external_torque = {0, 0, 0, 0, 0, 0, 0};
        while (isRuning)
        {
            auto t_start = high_resolution_clock::now();
            // get theroy torque
            for (int i = 0; i < _joint_num; ++i)
            {
                // Set initial joint positions
                joints(i) = getJointPosition(i);
                joint_velocities(i) = getJointVelocity(i); // Set initial joint velocities
                joint_accelerations(i) = 0;                // Set initial joint accelerations
            }
            rne_solver.CartToJnt(joints, joint_velocities, joint_accelerations, external_forces, joint_torques_rne);
            // get torque from sensor
            //joint_torques[joint_num] = filters[joint_num].filter(getJointLoadTorque(joint_num) / 1000 / Sensitivity[joint_num]) - ZeroOffset[joint_num] + joint_torques_rne(joint_num);
            joint_torques[joint_num] = getJointLoadTorque(joint_num)/ 1000 / Sensitivity[joint_num] - ZeroOffset[joint_num] + joint_torques_rne(joint_num);
            // std::cout<<"joint_torques[joint_num]"<<joint_torques[joint_num]<<std::endl;
            int j = joint_num;
            // 力矩计算

            // external_torque[j] = joint_torques[j];
            external_torque[j]=2;
            if (abs(external_torque[j]) - joint_dec[j] > 0)
            {
                // 模拟控制过程
                // 获取目标关节位置
                if (external_torque[j] < 0)
                {
                    external_torque[j] = external_torque[j] + joint_dec[j];
                        // external_torque[j]=-2;
                    std::cout << "力矩值为负 " << external_torque[j] << endl;
                }
                else if (external_torque[j] > 0)
                {

                    external_torque[j] = external_torque[j] - joint_dec[j];
                    // external_torque[j]=2;
                    std::cout << "力矩值为正 " << external_torque[j] << endl;
                }
            }
            else
            {
                external_torque[j] = 0;
                double update_velocity = 0;                    // Make sure to declare the appropriate data type
                joint_ctrl.updateVelocity(update_velocity, 5); // Make sure to call the appropriate function
            }

            std::vector<double> target_position = joint_ctrl.update(external_torque, dt_ctr);
            // std::cout << "q_target: ";
            for (int i = 0; i < _joint_num; i++)
            {
                q_target(i) = target_position[i];
                // std::cout << q_target(i) << " ";
            }
            // std::cout << std::endl;
            servoJ(q_target);
            outputFile << external_torque[j]<< std::flush;
            outputFile_ser<<q_target(joint_num)<<std::flush;
            outputFile_ser<<std::endl;
            outputFile<<std::endl;
            auto t_stop = high_resolution_clock::now();
            auto t_duration = std::chrono::duration<double>(t_stop - t_start);
            double double_t_duration=t_duration.count() ;
            if (double_t_duration< 0.001)
            {
                // std::cout << "Execution time: " <<double_t_duration<< " seconds" << std::endl;
                 std::this_thread::sleep_for(std::chrono::duration<double>(0.001 - t_duration.count()));
            }
            else
            {
                std::cout << "Execution time is greater than or equal to 1 millisecond." << std::endl;
            }
          
        }

        std::cout << "\nProgram exited." << std::endl;
    }

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
