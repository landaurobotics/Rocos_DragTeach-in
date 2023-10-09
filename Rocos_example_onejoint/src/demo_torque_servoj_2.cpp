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
#include <Eigen/Dense>
using namespace std::chrono;
DEFINE_string(urdf, "config/robot_onejoint.urdf", "Urdf file path");
DEFINE_string(base, "base_link", "Base link name");
DEFINE_string(tip, "link_1", "Tip link name");

KDL::Tree kdl_tree;
KDL::Chain robotChain;
bool isRuning = true;
// std::ofstream outputFile("/home/landau/Rocos_example/src/joint_torques.csv.csv");

std::array<Iir::Butterworth::LowPass<2>, 1> filter_array{};
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
            // outputFile.close();
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
    double averageFilter_JointTor(int &id, int &num)
    {
        int time_torque = 0;
        double torque;
        filter_array[id].setup(3, samplingrate, cutoff_frequency);
        while (time_torque < num)
        {
            torque = filter_array[id].filter(robot.getJointLoadTorque(id));
            // hw->waitForSignal( 0 );
            time_torque++;
        }

        return torque;
    }
    double averageFilter(int &id, int &num)
    {
        int time_torque = 0;
        double torque = 0.0;
        while (time_torque < num)
        {
            torque += robot.getJointLoadTorque(id);
            // hw->waitForSignal( 0 );
            time_torque++;
        }
        torque = torque / num;
        return torque;
    }
    double segmentedSigmoid(double x, double scale = 5, double shift = 23.0, double amplitude = 30.0)
    {
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

        int _joint_num = 1; /* Your joint number */

        std::string urdf_file = "/home/landau/Rocos_example/config/robot_onejoint.urdf";
        if (!kdl_parser::treeFromFile(urdf_file, kdl_tree)) // 建立tree
        {
            // 加载URDF文件失败，处理错误
            std::cout << "加载URDF文件失败" << std::endl;
            exit(0);
        }
        std::string base_link = "base_link"; // 设置基准链接
        std::string end_link = "link_1";     // 设置终端链接

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
        KDL::JntArray joint_real_torques(_joint_num);
        KDL::JntArray joint_cmd_torques(_joint_num);
        KDL::JntArray q_target1(_joint_num);
        KDL::JntArray q_target2(_joint_num);
        KDL::JntArray q_target3(_joint_num);
        KDL::JntArray q_target4(_joint_num);
        KDL::JntArray q_target5(_joint_num);
        KDL::JntArray q_target_mid(_joint_num);
        // Initialize joint positions, velocities, and accelerations (e.g., for a simple joint movement)

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

        //**-------------------------------**//

#pragma region //*电机使能检查
        // setJointMode(0, ModeOfOperation::CyclicSynchronousTorqueMode);

        // setJointTorque(0, 0);

        setEnabled();
#pragma endregion

        // double joint_tor[_joint_num]; // Assuming joint_tor is an array of doubles

        //** 移动均值滤波器初始化 **//
        const size_t filterSize = 10; // 滤波器窗口大小
        std::vector<MyFilter> filters(_joint_num, MyFilter(filterSize));
        //**-------------------------------**//
        //** 巴低通滤波器初始化 **//
        for (int i = 0; i < _joint_num; i++)
        {
            filter_array[i].setup(2, samplingrate, cutoff_frequency);
        }

        // Open a file for writing

        // if (outputFile.is_open())
        // {
        //     // Write the header
        //     outputFile << "Joint Torque" << std::endl;

        double joint_tor[_joint_num], filter_tor, compensation;

        double d_fext = 0.0;
        double fext = 0.0;
        double pos_prev = 0.0;
        double fext_prev = 0.0;
        double com_fext = 0.0;
        double pose = getJointPosition(0);
        double K = 0.01;
        double M = 8.0;
        double B = 2 * sqrt(M * K) * 16; //
        double acc = 0.0;
        double vel = 0.0;
        double dt = 0.001;
        double jerk = 0.0;
        int j = 0;
        double last_pose=pose;
        while (isRuning)
        {

            // auto us = chrono::duration_cast<chrono::microseconds>(chrono::system_clock::now().time_since_epoch());
            //  存入当前时刻
            //  outputFile << us.count() << ", ";
            //  // Your code to populate joint_tor array

            //
            for (int i = 0; i < _joint_num; ++i)
            {
                // Set initial joint positions
                joints(i) = getJointPosition(i);
                //std::cout << "pos: " << joints(i) << "\t";
                joint_velocities(i) = (joints(i) - pos_prev) / 0.001; // Set initial joint velocities
                joint_accelerations(i) = 0;                           // Set initial joint accelerations
                pos_prev = joints(i);
            }
            rne_solver.CartToJnt(joints, joint_velocities, joint_accelerations, external_forces, joint_torques);

            // for (int i = 0; i < _joint_num; ++i)
            // {
            //     joint_cmd_torques(i) = joint_torques(i) / ratio / torque_constant / peak_current * 1000;
            // }

            //

            // double Sensitivity[_joint_num] = {0.0066, 0.0069, 0.0110, 0.0106, 0.0101, 0.0241, 0.0241}; // V/eu
            double Sensitivity[_joint_num] = {0.006573};
            // double Capacity[_joint_num] = {-250, -250, -150, -150, -150, -70, -70};
            // double ZeroOffset[_joint_num] = {0.05, 0.0298, 0.016634, 0.014078, 0.018965, 0.011742, 0.038038}; // 单位v
            double ZeroOffset[_joint_num] = {0.07349};

            for (int i = 0; i < _joint_num; ++i)
            {

                // joint_tor = averageFilter_JointTor(i, Num_averge) / 1000 / Sensitivity[i];
                // joint_tor[i]=((getJointLoadTorque(i)/ 1000.0 )-ZeroOffset[i])/ Sensitivity[i];
                joint_tor[i] = filter_array[i].filter(((getJointLoadTorque(i) / 1000.0) - ZeroOffset[i]) / Sensitivity[i]);
                //std::cout << "load: " << joint_tor[i] << "\t";
                // joint_tor[i] = filters[i].filter(((getJointLoadTorque(i) / 1000.0) - ZeroOffset[i]) / Sensitivity[i]);
            }
            // outputFile << "1";
            
            if (j > 1000)
            {
                for (size_t i = 0; i < _joint_num; ++i)
                {
                    //std::cout << "theory: " << joint_torques(i) << "\t";
                    fext = joint_tor[i] - joint_torques(i);
                    // d_fext = fext - fext_prev;
                    com_fext = segmentedSigmoid(fext);

                    //std::cout << "com_fext: " << com_fext << "\t";
                    // if (abs(com_fext) > 0.01)
                    // {
                    //     acc = (com_fext - B * vel) / M;
                    //     // acc+=jerk*dt;
                    //     vel += acc * dt;

                    //     pose += vel * dt;
                    // }
                    // else
                    // {

                    //     acc = 0;
                    //     vel = vel/1.001;
                    //     pose += vel * dt;
                    // }
                    acc = (com_fext - B * vel) / M;
                    //     // acc+=jerk*dt;
                    vel += acc * dt;
                    pose += vel * dt;
                    if((abs(pose-last_pose))<0.00004)
                    {
                        pose=last_pose;
                    }
                    q_target1(i) = pose;
                    //std::cout << "com pose: " << q_target1(i);
                    last_pose=pose;
                }
                servoJ(q_target1);
            }

           
            j++;
            //std::cout << std::endl;
            // Sleep for a while before next iteration
            // robot.hw_interface_->waitForSignal(1);
            // std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Adjust the sleep duration as needed
        }

        // Close the file
        // outputFile.close();
        // std::cout << "Data saved to joint_torques.csv" << std::endl;
        // }
        // else
        // {
        //     std::cout << "Unable to open the file." << std::endl;
        // }
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

    // auto robotService = RobotServiceImpl::getInstance(&robot);

    //------------------------wait----------------------------------
    std::thread thread_test{&rocos::Robot::test, &robot};

    //------------------------wait----------------------------------
    // robotService->runServer();

    thread_test.join();

    return 0;
}
