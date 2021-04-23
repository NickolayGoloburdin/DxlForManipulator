usr/include/c++/9/bits/shared_ptr.h:717:39:   required from ‘std::shared_ptr<_Tp> std::make_shared(_Args&& ...) [with _Tp = ROSArm; _Args = {ROSArm}]’
/home/nuc/ros2_arm/src/arm_packet/src/arm_controll.cpp:89:54:   required from here
/usr/include/c++/9/ext/new_allocator.h:145:20: error: use of deleted function ‘ROSArm::ROSArm(ROSArm&&)’
  145 |  noexcept(noexcept(::new((void *)__p)
      |                    ^~~~~~~~~~~~~~~~~~
  146 |        _Up(std::forward<_Args>(__args)...)))
      |        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
In file included from /home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:1,
                 from /home/nuc/ros2_arm/src/arm_packet/src/arm_controll.cpp:9:
/home/nuc/ros2_arm/src/arm_packet/include/arm.hpp:96:7: note: ‘ROSArm::ROSArm(ROSArm&&)’ is implicitly deleted because the default definition would be ill-formed:
   96 | class ROSArm : public rclcpp::Node {
      |       ^~~~~~
/home/nuc/ros2_arm/src/arm_packet/include/arm.hpp:96:7: error: use of deleted function ‘rclcpp::Node::Node(const rclcpp::Node&)’
In file included from /home/nuc/ros2_rolling/install/rclcpp/include/rclcpp/context.hpp:33,
                 from /home/nuc/ros2_rolling/install/rclcpp/include/rclcpp/contexts/default_context.hpp:18,
                 from /home/nuc/ros2_rolling/install/rclcpp/include/rclcpp/executor.hpp:33,
                 from /home/nuc/ros2_rolling/install/rclcpp/include/rclcpp/executors/multi_threaded_executor.hpp:26,
                 from /home/nuc/ros2_rolling/install/rclcpp/include/rclcpp/executors.hpp:21,
                 from /home/nuc/ros2_rolling/install/rclcpp/include/rclcpp/rclcpp.hpp:156,
                 from /home/nuc/ros2_arm/src/arm_packet/include/arm.hpp:13,
                 from /home/nuc/ros2_arm/src/arm_packet/include/arm.cpp:1,
                 from /home/nuc/ros2_arm/src/arm_packet/src/arm_controll.cpp:9:
