# DxlForManipulator
In file included from /home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:1,
                 from /home/nuc/ros2_arm/src/arm_packet/src/arm_controll.cpp:9:
/home/nuc/ros2_arm/src/arm_packet/include/arm.hpp:104:21: error: ‘sensor_msgs’ was not declared in this scope
  104 |   rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisherjs_;
      |                     ^~~~~~~~~~~
/home/nuc/ros2_arm/src/arm_packet/include/arm.hpp:104:49: error: template argument 1 is invalid
  104 |   rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisherjs_;
      |                                                 ^
/home/nuc/ros2_arm/src/arm_packet/include/arm.hpp:104:52: error: expected ‘;’ at end of member declaration
  104 |   rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisherjs_;
      |                                                    ^~~~~~~~~
      |                                                             ;
/home/nuc/ros2_arm/src/arm_packet/include/arm.hpp:104:62: error: ‘publisherjs_’ does not name a type
  104 |   rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisherjs_;
      |                                                              ^~~~~~~~~~~~
/home/nuc/ros2_arm/src/arm_packet/include/arm.hpp:105:21: error: ‘sensor_msgs’ was not declared in this scope
  105 |   rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publishertl_;
      |                     ^~~~~~~~~~~
/home/nuc/ros2_arm/src/arm_packet/include/arm.hpp:105:49: error: template argument 1 is invalid
  105 |   rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publishertl_;
      |                                                 ^
/home/nuc/ros2_arm/src/arm_packet/include/arm.hpp:105:52: error: expected ‘;’ at end of member declaration
  105 |   rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publishertl_;
      |                                                    ^~~~~~~~~
      |                                                             ;
/home/nuc/ros2_arm/src/arm_packet/include/arm.hpp:105:52: error: redeclaration of ‘int ROSArm::SharedPtr’
/home/nuc/ros2_arm/src/arm_packet/include/arm.hpp:104:52: note: previous declaration ‘int ROSArm::SharedPtr’
  104 |   rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisherjs_;
      |                                                    ^~~~~~~~~
/home/nuc/ros2_arm/src/arm_packet/include/arm.hpp:105:62: error: ‘publishertl_’ does not name a type
  105 |   rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publishertl_;
      |                                                              ^~~~~~~~~~~~
/home/nuc/ros2_arm/src/arm_packet/include/arm.hpp:107:3: error: ‘sensor_msgs’ does not name a type
  107 |   sensor_msgs::msg::JointState joints_msg_;
      |   ^~~~~~~~~~~
/home/nuc/ros2_arm/src/arm_packet/include/arm.hpp:108:3: error: ‘sensor_msgs’ does not name a type
  108 |   sensor_msgs::msg::JointState temp_load_;
      |   ^~~~~~~~~~~
In file included from /home/nuc/ros2_arm/src/arm_packet/src/arm_controll.cpp:9:
/home/nuc/ros2_arm/src/arm_packet/include/arm.cpp: In member function ‘void Manipulator::set_start_pos()’:
/home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:5:13: warning: unused variable ‘i’ [-Wunused-variable]
    5 |   for (auto i : dxl_id)
      |             ^
/home/nuc/ros2_arm/src/arm_packet/include/arm.cpp: In member function ‘void Manipulator::arm_init()’:
/home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:18:5: error: ‘ROS_ERROR_STREAM’ was not declared in this scope; did you mean ‘RCLCPP_ERROR_STREAM’?
   18 |     ROS_ERROR_STREAM(log);
      |     ^~~~~~~~~~~~~~~~
      |     RCLCPP_ERROR_STREAM
/home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:23:47: error: invalid operands of types ‘const char [21]’ and ‘int’ to binary ‘operator<<’
   23 |       ROS_ERROR_STREAM("Failed to init motor" << i + 1);
      |                        ~~~~~~~~~~~~~~~~~~~~~~ ^~ ~~~~~
      |                        |                           |
      |                        const char [21]             int
/home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:23:52: warning: suggest parentheses around ‘+’ inside ‘<<’ [-Wparentheses]
   23 |       ROS_ERROR_STREAM("Failed to init motor" << i + 1);
      |                                                  ~~^~~
/home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:23:7: error: ‘ROS_ERROR_STREAM’ was not declared in this scope; did you mean ‘RCLCPP_ERROR_STREAM’?
   23 |       ROS_ERROR_STREAM("Failed to init motor" << i + 1);
      |       ^~~~~~~~~~~~~~~~
      |       RCLCPP_ERROR_STREAM
/home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:28:7: error: ‘ROS_ERROR_STREAM’ was not declared in this scope; did you mean ‘RCLCPP_ERROR_STREAM’?
   28 |       ROS_ERROR_STREAM(log);
      |       ^~~~~~~~~~~~~~~~
      |       RCLCPP_ERROR_STREAM
In file included from /home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:1,
                 from /home/nuc/ros2_arm/src/arm_packet/src/arm_controll.cpp:9:
/home/nuc/ros2_arm/src/arm_packet/include/arm.hpp: In constructor ‘ROSArm::ROSArm(Manipulator*)’:
/home/nuc/ros2_arm/src/arm_packet/include/arm.hpp:106:10: warning: ‘ROSArm::count_’ will be initialized after [-Wreorder]
  106 |   size_t count_;
      |          ^~~~~~
/home/nuc/ros2_arm/src/arm_packet/include/arm.hpp:99:16: warning:   ‘Manipulator* ROSArm::manip_’ [-Wreorder]
   99 |   Manipulator *manip_;
      |                ^~~~~~
In file included from /home/nuc/ros2_arm/src/arm_packet/src/arm_controll.cpp:9:
/home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:131:1: warning:   when initialized here [-Wreorder]
  131 | ROSArm::ROSArm(Manipulator *manip)
      | ^~~~~~
/home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:135:3: error: ‘publisherjs_’ was not declared in this scope
  135 |   publisherjs_ = this->create_publisher<sensor_msgs::msg::JointState>(
      |   ^~~~~~~~~~~~
/home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:135:41: error: ‘sensor_msgs’ was not declared in this scope
  135 |   publisherjs_ = this->create_publisher<sensor_msgs::msg::JointState>(
      |                                         ^~~~~~~~~~~
/home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:135:24: error: parse error in template argument list
  135 |   publisherjs_ = this->create_publisher<sensor_msgs::msg::JointState>(
      |                        ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:136:29: error: no matching function for call to ‘ROSArm::create_publisher<<expression error> >(const char [17], int)’
  136 |       "arm_joint_states", 64);
      |                             ^
In file included from /home/nuc/ros2_rolling/install/rclcpp/include/rclcpp/executors/single_threaded_executor.hpp:28,
                 from /home/nuc/ros2_rolling/install/rclcpp/include/rclcpp/executors.hpp:22,
                 from /home/nuc/ros2_rolling/install/rclcpp/include/rclcpp/rclcpp.hpp:156,
                 from /home/nuc/ros2_arm/src/arm_packet/include/arm.hpp:13,
                 from /home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:1,
                 from /home/nuc/ros2_arm/src/arm_packet/src/arm_controll.cpp:9:
/home/nuc/ros2_rolling/install/rclcpp/include/rclcpp/node.hpp:189:3: note: candidate: ‘template<class MessageT, class AllocatorT, class PublisherT> std::shared_ptr<PublisherT> rclcpp::Node::create_publisher(const string&, const rclcpp::QoS&, const rclcpp::PublisherOptionsWithAllocator<AllocatorT>&)’
  189 |   create_publisher(
      |   ^~~~~~~~~~~~~~~~
/home/nuc/ros2_rolling/install/rclcpp/include/rclcpp/node.hpp:189:3: note:   template argument deduction/substitution failed:
In file included from /home/nuc/ros2_arm/src/arm_packet/src/arm_controll.cpp:9:
/home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:136:29: error: template argument 1 is invalid
  136 |       "arm_joint_states", 64);
      |                             ^
/home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:137:3: error: ‘publishertl_’ was not declared in this scope
  137 |   publishertl_ =
      |   ^~~~~~~~~~~~
/home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:138:13: error: parse error in template argument list
  138 |       this->create_publisher<sensor_msgs::msg::JointState>("temp_load", 64);
      |             ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:138:75: error: no matching function for call to ‘ROSArm::create_publisher<sensor_msgs>(const char [10], int)’
  138 |       this->create_publisher<sensor_msgs::msg::JointState>("temp_load", 64);
      |                                                                           ^
In file included from /home/nuc/ros2_rolling/install/rclcpp/include/rclcpp/executors/single_threaded_executor.hpp:28,
                 from /home/nuc/ros2_rolling/install/rclcpp/include/rclcpp/executors.hpp:22,
                 from /home/nuc/ros2_rolling/install/rclcpp/include/rclcpp/rclcpp.hpp:156,
                 from /home/nuc/ros2_arm/src/arm_packet/include/arm.hpp:13,
                 from /home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:1,
                 from /home/nuc/ros2_arm/src/arm_packet/src/arm_controll.cpp:9:
/home/nuc/ros2_rolling/install/rclcpp/include/rclcpp/node.hpp:189:3: note: candidate: ‘template<class MessageT, class AllocatorT, class PublisherT> std::shared_ptr<PublisherT> rclcpp::Node::create_publisher(const string&, const rclcpp::QoS&, const rclcpp::PublisherOptionsWithAllocator<AllocatorT>&)’
  189 |   create_publisher(
      |   ^~~~~~~~~~~~~~~~
/home/nuc/ros2_rolling/install/rclcpp/include/rclcpp/node.hpp:189:3: note:   template argument deduction/substitution failed:
In file included from /home/nuc/ros2_arm/src/arm_packet/src/arm_controll.cpp:9:
/home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:140:31: error: unable to find numeric literal operator ‘operator""ms’
  140 |       this->create_wall_timer(500ms, std::bind(&ROSArm::timer_callback, this));
      |                               ^~~~~
/home/nuc/ros2_arm/src/arm_packet/include/arm.cpp: In member function ‘void ROSArm::timer_callback()’:
/home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:148:3: error: ‘joints_msg_’ was not declared in this scope
  148 |   joints_msg_.velocity = manip_->present_speed;
      |   ^~~~~~~~~~~
/home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:150:30: error: ‘ros’ has not been declared
  150 |   joints_msg_.header.stamp = ros::Time::now();
      |                              ^~~
/home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:151:3: error: ‘publisherjs_’ was not declared in this scope
  151 |   publisherjs_->publish(joint_msg_);
      |   ^~~~~~~~~~~~
/home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:151:25: error: ‘joint_msg_’ was not declared in this scope
  151 |   publisherjs_->publish(joint_msg_);
      |                         ^~~~~~~~~~
/home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:153:3: error: ‘temp_load_’ was not declared in this scope
  153 |   temp_load_.velocity = manip->present_load;
      |   ^~~~~~~~~~
/home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:153:25: error: ‘manip’ was not declared in this scope; did you mean ‘manip_’?
  153 |   temp_load_.velocity = manip->present_load;
      |                         ^~~~~
      |                         manip_
/home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:155:29: error: ‘ros’ has not been declared
  155 |   temp_load_.header.stamp = ros::Time::now();
      |                             ^~~
/home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:156:3: error: ‘publishertl_’ was not declared in this scope
  156 |   publishertl_->publish(temp_load_);
      |   ^~~~~~~~~~~~
/home/nuc/ros2_arm/src/arm_packet/src/arm_controll.cpp: In function ‘int main(int, char**)’:
/home/nuc/ros2_arm/src/arm_packet/src/arm_controll.cpp:90:33: error: temporary of non-literal type ‘ROSArm’ in a constant expression
   90 |   rclcpp::spin(std::make_shared<ROSArm(manip)>(ROSArm(manip)));
      |                                 ^~~~~~~~~~~~~
In file included from /home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:1,
                 from /home/nuc/ros2_arm/src/arm_packet/src/arm_controll.cpp:9:
/home/nuc/ros2_arm/src/arm_packet/include/arm.hpp:95:7: note: ‘ROSArm’ is not literal because:
   95 | class ROSArm : public rclcpp::Node {
      |       ^~~~~~
/home/nuc/ros2_arm/src/arm_packet/include/arm.hpp:95:7: note:   ‘ROSArm’ has a non-trivial destructor
/home/nuc/ros2_arm/src/arm_packet/src/arm_controll.cpp:90:61: error: no matching function for call to ‘make_shared<ROSArm(manip)>(ROSArm)’
   90 |   rclcpp::spin(std::make_shared<ROSArm(manip)>(ROSArm(manip)));
      |                                                             ^
In file included from /usr/include/c++/9/memory:81,
                 from /usr/include/boost/config/no_tr1/memory.hpp:21,
                 from /usr/include/boost/get_pointer.hpp:14,
                 from /usr/include/boost/bind/mem_fn.hpp:25,
                 from /usr/include/boost/mem_fn.hpp:22,
                 from /usr/include/boost/bind/bind.hpp:26,
                 from /usr/include/boost/bind.hpp:22,
                 from /home/nuc/ros2_arm/src/arm_packet/include/arm.hpp:9,
                 from /home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:1,
                 from /home/nuc/ros2_arm/src/arm_packet/src/arm_controll.cpp:9:
/usr/include/c++/9/bits/shared_ptr.h:714:5: note: candidate: ‘template<class _Tp, class ... _Args> std::shared_ptr<_Tp> std::make_shared(_Args&& ...)’
  714 |     make_shared(_Args&&... __args)
      |     ^~~~~~~~~~~
/usr/include/c++/9/bits/shared_ptr.h:714:5: note:   template argument deduction/substitution failed:
make[2]: *** [CMakeFiles/arm_controll.dir/build.make:63: CMakeFiles/arm_controll.dir/src/arm_controll.cpp.o] Error 1
make[1]: *** [CMakeFiles/Makefile2:78: CMakeFiles/arm_controll.dir/all] Error 2
make: *** [Makefile:141: all] Error 2
---
Failed   <<< arm_packet [4.54s, exited with code 2]
Aborted  <<< common_interfaces [2.55s]
