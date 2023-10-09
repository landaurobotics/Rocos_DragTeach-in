#include <iostream>
#include <cmath>
#include <vector>
#include <chrono>
#include <thread>
#include <csignal>
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

DEFINE_string(urdf, "config/robot.urdf", "Urdf file path");
DEFINE_string(base, "base_link", "Base link name");
DEFINE_string(tip, "link_7", "Tip link name");

bool isRuning = true;

using namespace std::chrono_literals;


const double PI = 3.14159265358979323846;

class JointControl {
public:
    JointControl(const std::vector<double>& joint_mass, const std::vector<double>& joint_damping,
                 const std::vector<double>& joint_stiffness, const std::vector<double>& max_velocity,
                 const std::vector<double>& max_acceleration, const std::vector<double>& init_pose)
        : joint_mass_(joint_mass), joint_damping_(joint_damping), joint_stiffness_(joint_stiffness),
          max_velocity_(max_velocity), max_acceleration_(max_acceleration), current_position_(init_pose),
          current_velocity_(std::vector<double>(6, 0.0)) {}

    void updateVelocity(double velocity, int num) {
        current_velocity_[num] = velocity;
    }

    std::vector<double> update(const std::vector<double>& external_torque, double dt) {
        for (int i = 0; i < 6; ++i) {
            double current_acceleration =
                (external_torque[i] - joint_damping_[i] * current_velocity_[i]) / joint_mass_[i];
            current_acceleration = std::max(-max_acceleration_[i], std::min(max_acceleration_[i], current_acceleration));
            
            current_velocity_[i] += current_acceleration * dt;
            current_velocity_[i] = std::max(-max_velocity_[i], std::min(max_velocity_[i], current_velocity_[i]));
            
            current_position_[i] += current_velocity_[i] * dt;
            
            if (current_position_[i] < -180.0 * PI / 180.0 || current_position_[i] > 180.0 * PI / 180.0) {
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

volatile std::sig_atomic_t g_signal_status = 0;

void signalHandler(int signal) {
    g_signal_status = signal;
}

int main() {
    std::signal(SIGINT, signalHandler);

    

    std::vector<double> joint_mass = {10, 20, 6, 3, 2, 1};
    std::vector<double> joint_damping = {2, 15, 10, 5, 1.5, 1};
    std::vector<double> joint_stiffness = {10.0, 10, 10, 10, 10, 10};
    std::vector<double> max_velocity = {0.1, 0.1, 0.1, 0.2, 0.2, 0.2};
    std::vector<double> max_acceleration = {0.1, 0.1, 0.1, 0.2, 0.2, 0.2};
    // std::vector<double> init_pose = rtde_receive.getTargetQ();

    double dt_ctr = 0.01;

    double velocity = 0.2;
    double acceleration = 0.2;
    double dt = 1.0 / 500.0;
    double lookahead_time = 0.15;
    double gain = 300.0;
    // std::vector<double> joint_q = rtde_receive.getActualQ();

    JointControl joint_ctrl(joint_mass, joint_damping, joint_stiffness, max_velocity, max_acceleration, init_pose);

    while (g_signal_status == 0) {
        // std::vector<double> joint_torques = rtde_control.getJointTorques();
        std::cout << "Compensated joint torques: ";
        for (const auto& torque : joint_torques) {
            std::cout << torque << " ";
        }
        std::cout << std::endl;

        std::vector<double> external_torque = joint_torques;
        // Perform torque processing and control logic here
        // ...

        joint_ctrl.updateVelocity(rtde_receive.getActualQd()[joint_num], joint_num);
        std::vector<double> target_position = joint_ctrl.update(external_torque, dt_ctr);

        auto t_start = rtde_control.initPeriod();
        rtde_control.servoJ(joint_q, velocity, acceleration, dt, lookahead_time, gain);
        joint_q = target_position;
        rtde_control.waitPeriod(t_start);
    }

    std::cout << "\nProgram exited." << std::endl;

    return 0;
}