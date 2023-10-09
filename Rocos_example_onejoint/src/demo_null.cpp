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

#include <rocos_app/drive.h>
#include <rocos_app/ethercat/hardware.h>
#include <rocos_app/ethercat/hardware_sim.h>
#include <fstream>
#include <iostream>
#include <rocos_app/robot.h>
#include <rocos_app/robot_service.h>
#include <string>
#include <gflags/gflags.h>

DEFINE_string(urdf, "config/robot.urdf", "Urdf file path");
DEFINE_string(base, "base_link", "Base link name");
DEFINE_string(tip, "link_7", "Tip link name");
std::atomic<bool> *NullBoolPtr = new std::atomic<bool>(false);
bool isRuning = true;
// using namespace rocos;

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
    //boost::shared_ptr<HardwareInterface> hw = boost::make_shared<HardwareSim>(_joint_num); // 仿真
    boost::shared_ptr<HardwareInterface> hw = boost::make_shared<Hardware>(); // 真实机械臂

    Robot *robot = new Robot(hw, FLAGS_urdf, FLAGS_base, FLAGS_tip);

    void signalHandler(int signo)
    {
        if (signo == SIGINT)
        {
            std::cout << "\033[1;31m"
                      << "[!!SIGNAL!!]"
                      << "INTERRUPT by CTRL-C"
                      << "\033[0m" << std::endl;

            isRuning = false;
            robot->setDisabled();
            exit(0);
        }
    }

    void Robot::test()
    {

        robot->setEnabled();
        KDL::JntArray q_target1(_joint_num);
        // q_target1(0) = 0 * M_PI / 180;
        // q_target1(1) = 0 * M_PI / 180;
        // q_target1(2) = 0 * M_PI / 180;
        // q_target1(3) = 0 * M_PI / 180;
        // q_target1(4) = 0 * M_PI / 180;
        // q_target1(5) = 0 * M_PI / 180;
        // q_target1(6) = 0 * M_PI / 180;
        // MoveJ(q_target1, 0.4, 4, 0, 0, false);
        q_target1(0) = 0 * M_PI / 180;
        q_target1(1) = 45 * M_PI / 180;
        q_target1(2) = 0 * M_PI / 180;
        q_target1(3) = 90 * M_PI / 180;
        q_target1(4) = 0 * M_PI / 180;
        q_target1(5) = -45 * M_PI / 180;
        q_target1(6) = 0 * M_PI / 180;

        
        sleep(1);
        // for ( int i = 0; i < 140; i++ )
        //    {
        //     robot->Dragging(DRAGGING_FLAG::NULLSPACE, DRAGGING_DIRRECTION::POSITION, 0.4, 2);
        //     std::this_thread::sleep_for( std::chrono::milliseconds( 30 ) );
        //    }

        while (isRuning)
        {
            // std::string str;
            // PLOG_INFO << "当前环境是否安全,如果是,输入run开始执行程序";
            // std::cin >> str;

            // if (str == std::string_view{"run"})
            // {
            MoveJ(q_target1, 0.2, 1, 0, 0, false);
            sleep(1);

            for (int i = 0; i < 100; i++)
            {
                robot->Dragging(DRAGGING_FLAG::NULLSPACE, DRAGGING_DIRRECTION::NEGATIVE, 0.8, 2);
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
            }

            sleep(1);

            for (int i = 0; i < 200; i++)
            {
                robot->Dragging(DRAGGING_FLAG::NULLSPACE, DRAGGING_DIRRECTION::POSITION, 0.8, 2);
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
            }

            sleep(1);
            for (int i = 0; i < 100; i++)
            {
                robot->Dragging(DRAGGING_FLAG::NULLSPACE, DRAGGING_DIRRECTION::NEGATIVE, 0.8, 2);
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
            }
            sleep(4);
            std::cout << "零空间运动已完成" << std::endl;
            MoveJ(q_target1, 0.2, 1, 0, 0, false);
            // movel
            KDL::Frame f_p0;
            KDL::Frame f_p1;
            KDL::Frame f_p2;
            KDL::Frame f_p3;
            KDL::Frame f_p4;
            KDL::Frame f_p5;
            KDL::Frame f_p6;
            kinematics_.JntToCart(q_target1, f_p0);
            f_p1 = f_p0 * KDL::Frame{KDL::Vector{0.0, 0.15, 0.0}};
            // MoveL( f_p1, 0.1, 0.2, 0, 0, false );
            f_p2 = f_p1 * KDL::Frame{KDL::Vector{0.3, 0.0, 0.0}};
            // MoveL( f_p2, 0.1, 0.2, 0, 0, false );
            f_p3 = f_p2 * KDL::Frame{KDL::Vector{0.0, -0.3, 0.0}};
            // MoveL( f_p3, 0.1, 0.2, 0, 0, false );
            f_p4 = f_p3 * KDL::Frame{KDL::Vector{-0.3, 0, 0.0}};
            f_p5 = f_p4 * KDL::Frame{KDL::Vector{0, 0.15, 0.0}};
            // MoveL( f_p4, 0.1, 0.2, 0, 0, false );

            // MoveJ( q_target1, 0.2, 0.2, 0, 0, false );
            std::vector<KDL::Frame> target_pose;
            std::vector<double> bound_list = {0.2, 0.2, 0.2, 0.2, 0.2};
            std::vector<double> max_path_v = {0.1, 0.1, 0.1, 0.1, 0.1};
            std::vector<double> max_path_a = {0.4, 0.4, 0.4, 0.4, 0.4};

            target_pose.push_back(f_p1);
            target_pose.push_back(f_p2);
            target_pose.push_back(f_p3);
            target_pose.push_back(f_p4);
            target_pose.push_back(f_p5);
            MultiMoveL(target_pose, bound_list, max_path_v, max_path_a);
            sleep(1);
        }

        // if (flag)
        // {
        //     std::cout << "零空间运动已完成" << std::endl;
        // }
        // else
        // {
        //     std::cout << "零空间运动被中断" << std::endl;
        // }

        // int i = 0;
        // while (i < 1)

        // {
        //     using namespace KDL;
        //     KDL::JntArray q_target1(_joint_num);
        //     KDL::JntArray q_target2(_joint_num);
        //     KDL::JntArray q_target3(_joint_num);
        //     KDL::JntArray q_target4(_joint_num);
        //     double v_demo = 0.1;
        //     double acc_demo = 2;
        //     //** movej 点位1**//
        //     q_target1(0) = 90 * M_PI / 180;
        //     q_target1(1) = 90 * M_PI / 180;
        //     q_target1(2) = 0 * M_PI / 180;
        //     q_target1(3) = 0 * M_PI / 180;
        //     q_target1(4) = 0 * M_PI / 180;
        //     q_target1(5) = 0 * M_PI / 180;
        //     q_target1(6) = 0 * M_PI / 180;

        //     MoveJ(q_target1, v_demo, acc_demo, 0, 0, false);
        //     //** movej 点位2**//
        //     q_target2(0) = -90 * M_PI / 180;
        //     q_target2(1) = 90 * M_PI / 180;
        //     q_target2(2) = 0 * M_PI / 180;
        //     q_target2(3) = 0 * M_PI / 180;
        //     q_target2(4) = 0 * M_PI / 180;
        //     q_target2(5) = 0 * M_PI / 180;
        //     q_target2(6) = 0 * M_PI / 180;

        //     MoveJ(q_target2, v_demo, acc_demo, 0, 0, false);

        //     //** movej 点位3**//
        //     q_target3(0) = -90 * M_PI / 180;
        //     q_target3(1) = -90 * M_PI / 180;
        //     q_target3(2) = 0 * M_PI / 180;
        //     q_target3(3) = 0 * M_PI / 180;
        //     q_target3(4) = 0 * M_PI / 180;
        //     q_target3(5) = 0 * M_PI / 180;
        //     q_target3(6) = 0 * M_PI / 180;

        //     MoveJ(q_target3, v_demo, acc_demo, 0, 0, false);

        //     //** movej 点位4**//
        //     q_target4(0) = 90 * M_PI / 180;
        //     q_target4(1) = -90 * M_PI / 180;
        //     q_target4(2) = 0 * M_PI / 180;
        //     q_target4(3) = 0 * M_PI / 180;
        //     q_target4(4) = 0 * M_PI / 180;
        //     q_target4(5) = 0 * M_PI / 180;
        //     q_target4(6) = 0 * M_PI / 180;

        //     MoveJ(q_target4, v_demo, acc_demo, 0, 0, false);
        //     i += 1;
        //     std::cout << "运行次数= " << i << std::endl;
        // }
        //**-------------------------------**//

        //** Multimovel **//

        //** sin **//

        //** movej **//
        //                KDL::JntArray q_target(_joint_num);
        //                q_target(0) = 0 * M_PI / 180;
        //                q_target(1) = -90 * M_PI / 180;
        //                q_target(2) = 55 * M_PI / 180;
        //                q_target(3) = 90 * M_PI / 180;
        //                q_target(4) = 40 * M_PI / 180;
        //                q_target(5) = -45 * M_PI / 180;
        //                q_target(6) = 0 * M_PI / 180;

        //                MoveJ(q_target, 0.8, 0.6, 0, 0, false);

        //                KDL::Frame f_p0;
        //                kinematics_.JntToCart(q_target, f_p0);
        //                //** servoL **//
        //                KDL::Frame f_p5;
        //                f_p5 = f_p0;
        //                double i = 0;
        //                while (i < KDL::PI)
        //                {
        //                    f_p5.p(2) = f_p0.p(2) + 0.1 * sin(3 * i);
        //                    servoL(f_p5);
        //                    i = i + 0.001;
        //                }

        //**-------------------------------**//
        // }
        // else
        // {
        //     PLOG_ERROR << "不安全环境,电机抱闸";
        //     setDisabled();

        //     return;
        // }
        // }
        // std::cout << "Code is end" << std::endl;
        // setDisabled();
    }

    // PLOG_INFO << "全部测试结束,goodbye!";
}
// namespace rocos
#pragma endregion

/// \brief 处理终端的Ctrl-C信号
/// \param signo
// void signalHandler(int signo)
// {
//     if (signo == SIGINT)
//     {
//         std::cout << "\033[1;31m"
//                   << "[!!SIGNAL!!]"
//                   << "INTERRUPT by CTRL-C"
//                   << "\033[0m" << std::endl;

//         isRuning = false;

//         exit(0);
//     }
// }
int set_process_priority_max(pid_t pid)
{
    const int max_thread_priority = sched_get_priority_max(SCHED_FIFO);
    if (max_thread_priority != -1)
    {
        // We'll operate on the currently running thread.
        // pthread_t this_thread = pthread_self( );

        // struct sched_param is used to store the scheduling priority
        struct sched_param params;

        params.sched_priority = max_thread_priority;

        int ret = sched_setscheduler(pid, SCHED_FIFO, &params);
        if (ret != 0)
        {
            std::cerr << RED << "Unsuccessful in setting main process realtime priority. Error code: " << ret << GREEN << std::endl;
            return -1;
        }
        // Now verify the change in thread priority
        ret = sched_getparam(pid, &params);
        if (ret != 0)
        {
            std::cerr << RED << "Couldn't retrieve real-time scheduling paramers" << GREEN << std::endl;
            return -1;
        }

        // Print thread scheduling priority
        std::cout << GREEN << "Main process : [" << pid << "] priority is " << params.sched_priority << std::endl;
        return 0;
    }
    else
    {
        std::cerr << RED << "Could not get maximum thread priority for main process" << GREEN << std::endl;
        return -1;
    }
}
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

    std::cout << FLAGS_urdf << std::endl;

    auto robotService = RobotServiceImpl::getInstance(robot);

    //------------------------wait----------------------------------
    std::thread thread_test{&rocos::Robot::test, robot};

    //------------------------wait----------------------------------
    robotService->runServer();

    thread_test.join();

    return 0;
}
