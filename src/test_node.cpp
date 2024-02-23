#include <iostream>

// ROS includes
#include "rclcpp/rclcpp.hpp"

// UR driver:
#include "ioc_ur_cb2_driver/ur_driver.h"


class TestNode : public rclcpp::Node {
public:
  // Constructor:
  TestNode() : Node("test_node"){
    // pub_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  }

private:
  // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  // Creates a shared pointer to an instance of the TestNode class:
  auto my_TestNode = std::make_shared<TestNode>();

  // Main variables:
  std::unique_ptr<UrDriver> ur_driver_;
  unsigned int menu_selection = 0;
  bool stop = false;
  std::vector<double> q;
  std::string cmd;

  // Welcome:
  std::cout << std::endl;
  std::cout << "----- IOC-UR-CB2-DRIVER TEST -----" << std::endl;
  std::cout << std::endl;

  // Initialize the driver:
  std::condition_variable rt_msg_cond_;
  std::condition_variable msg_cond_;
  std::string robot_ip_default = "192.168.11.2";
  std::string robot_ip;
  int reverse_port = 50001;
  std::cout << "Follow the steps to initialize the ioc_ur_cb2_driver:" << std::endl;
  std::cout << "Enter the robot IP (default=" << robot_ip_default << "): ";
  std::getline(std::cin, robot_ip);
  if (robot_ip.empty()) {
      robot_ip = robot_ip_default;
  }	

  ur_driver_ = std::make_unique<UrDriver>(rt_msg_cond_, msg_cond_, robot_ip, reverse_port, 0.03, 300);

  std::mutex msg_lock;
  std::unique_lock<std::mutex> locker(msg_lock);

  if (ur_driver_->start()) {
    std::cout << "INFO: ioc_ur_cb2_driver initialized successfully" << std::endl;
    // BEGIN LOOP
    while(!stop){
      // Print menu:
      std::cout<<std::endl;
      std::cout << "Please, select an option to continue: " << std::endl;
      std::cout << "     <0> EXIT" << std::endl;
      std::cout << "     <1> Get actual joints states" << std::endl;
      std::cout << "     <2> Set joint positions" << std::endl;
      std::cout << "     <3> Set joint velocities" << std::endl;
      std::cout<<std::endl;
      // Get menu selection:
      std::cout << "Enter your selection: ";
      std::cin >> menu_selection;
      std::cout << std::endl;
      // Execute selection:
      switch(menu_selection){
        case 0: // Stop:
          ur_driver_->halt();
          std::cout << "Exiting the driver test" << std::endl;
          stop = true;
          break;

        case 1: // Read joints:
          while (!ur_driver_->rt_interface_->robot_state_->getDataPublished()) {
              rt_msg_cond_.wait(locker);
          }
          q = ur_driver_->rt_interface_->robot_state_->getQActual();
          for(unsigned int i=0;i < q.size();i++){
              std::cout << "q[" << i << "]= " << q[i] << std::endl;;
          }   
          ur_driver_->rt_interface_->robot_state_->setDataPublished();
          break;

        case 2: // Set positions:
          cmd = "servoj([1.0,-0.7,2.0,-2.0,-0.8,3.0],0,0,0.02)";   //(q[],X,X,t)
          std::cout << "Moving to the hardcoded joints positions..." << std::endl;
          for(unsigned int k=0; k < 1000; k++){
              ur_driver_->rt_interface_->addCommandToQueue(cmd);
          }
          break;

        case 3: // Set velocities:
          std::cout << "Moving by velocity the hardcoded joints..." << std::endl;
          for(unsigned int k=0; k < 1000; k++){
              ur_driver_->setSpeed(0.0,0.0,0.0,0.0,0.0,-0.1,5.0);
          }
          break;

        default:
          std::cout << "Invalid choice!" << std::endl;
          break;
      } // END SWITCH-CASE
    } // END LOOP
  } else {
      std::cout << "ERROR: ioc_ur_cb2_driver NOT initialized successfully" << std::endl;
  }

  // rclcpp::spin(my_TestNode);
  rclcpp::shutdown();
  return 0;
}