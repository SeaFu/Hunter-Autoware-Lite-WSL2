/*
 * demo_hunter_can.cpp
 *
 * Created on: Jun 12, 2019 05:03
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

// #include "ugv_sdk/hunter/hunter_base.hpp"
//
// using namespace westonrobot;
//
// int main(int argc, char **argv) {
//   HunterBase hunter;
//   hunter.Connect("can0");
//
//   int count = 0;
//   while (true) {
//     auto state = hunter.GetHunterState();
//     std::cout << "-------------------------------" << std::endl;
//     std::cout << "count: " << count << std::endl;
//     std::cout << "control mode: " << static_cast<int>(state.control_mode)
//               << " , base state: " << static_cast<int>(state.base_state)
//               << std::endl;
//     std::cout << "parking mode: " << static_cast<int>(state.park_mode)
//               << std::endl;
//     std::cout << "battery voltage: " << state.battery_voltage << std::endl;
//     std::cout << "velocity (linear, angular): " << state.linear_velocity << ", "
//               << state.steering_angle << std::endl;
//     for (int i = 0; i < 3; ++i)
//       std::cout << "motor rpm " << i << ": " << state.actuator_states[i].motor_rpm
//                 << std::endl;
//     std::cout << "-------------------------------" << std::endl;
//
//     sleep(1);
//     ++count;
//   }
//
//   return 0;
// }

#include "hunter_base/hunter_messenger.hpp"

using namespace westonrobot;

std::unique_ptr<HunterRobot> robot;

int main(int argc, char **argv) {

  int version = 2;
  std::string port_name = "can0";

  ProtocolDetector detector;
  try
  {
      detector.Connect(port_name);
      auto proto = detector.DetectProtocolVersion(5);
      if (proto == ProtocolVersion::AGX_V1) {
        std::cout << "Detected protocol: AGX_V1" << std::endl;
        robot = std::unique_ptr<HunterRobot>(
        new HunterRobot(ProtocolVersion::AGX_V1));
        version = 1;
      } else if (proto == ProtocolVersion::AGX_V2) {
        std::cout << "Detected protocol: AGX_V2" << std::endl;
        robot = std::unique_ptr<HunterRobot>(
        new HunterRobot(ProtocolVersion::AGX_V2));
      } else {
        std::cout << "Detected protocol: UNKONWN" << std::endl;
        return -1;
      }
      if (robot == nullptr)
        std::cout << "Failed to create robot object" << std::endl;
  }
  catch (std::exception error)
  {
      std::cerr << ("please bringup up can or make sure can port exist") << std::endl;
  }

  // connect to robot and setup ROS subscription
  if (port_name.find("can") != std::string::npos) {
    robot->Connect(port_name);
    robot->EnableCommandedMode();
    ROS_INFO("Using CAN bus to talk with the robot: %s", port_name.c_str());
  } else {
    ROS_ERROR("Only CAN bus interface is supported for now");
    return -1;
  }

  int count = 0;

  // HunterRobot *hunter = robot.get();
  while (true) {
    HunterRobot *hunter = robot.get();
    auto state = hunter->GetRobotState();

    // std::cout << "-------------------------------" << std::endl;
    // std::cout << "count: " << count << std::endl;
    // std::cout << "control mode: " << static_cast<int>(state.control_mode)
    //           << " , base state: " << static_cast<int>(state.base_state)
    //           << std::endl;
    // std::cout << "parking mode: " << static_cast<int>(state.park_mode)
    //           << std::endl;
    // std::cout << "battery voltage: " << state.battery_voltage << std::endl;
    std::cout << "velocity (linear, angular): " << state.motion_state.linear_velocity << ", "
              << state.motion_state.steering_angle << std::endl;
    // for (int i = 0; i < 3; ++i)
    //   std::cout << "motor rpm " << i << ": " << state.actuator_states[i].motor_rpm
    //             << std::endl;
    // std::cout << "-------------------------------" << std::endl;

    sleep(1);
    ++count;
  }

  return 0;
}
