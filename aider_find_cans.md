# ROS2 Jazzy Node: Find cans in lidar scandata

Ingest the information from this file, implement the low-level tasks, and generate
the code that will satisfy the high and mid-level objectives

## High-level Objective

Add a laser scan data analysis node to the directory structure. The node is to find
soda cans in the incoming laser scan data.

## Mid-level Objective

The node is to be a ROS2 publisher and subscriber. It shall analyze the
laser scan data and identify the soda cans it finds in the laser scan
data and output the position of the closest can relative to the laser scanner. It shall
also publish a sensor_msgs/msg/LaserScan message to topic '/can_scan' that contains only
points which are on a can.
The node will identify which points are on a can using a filter described later.
It will also publish the closest found can location to /goal_can_pos using message type
geometry_msgs/msg/PointStamped

## Implementation Notes
- Be sure to follow the low-level tasks in order and in detail
- Use the current git branch
- This will be a C++ node
- add DocStrings;
- comment the code;

## context

### Previously executed commands

```bash
cd ~/ros2_ws/src
# create a new package for the scan analyzer
ros2 pkg create --build-type ament_cmake --license Apache-2.0 --node-name can_finder can_finder
# Build the workspace
cd ~/ros2_ws
colcon build --symlink-install
```

### beginning context

/add src/ScanAnalyser.cpp
- The diameter of a can is 0.066. 
- The angle theta subtended by a can at range r is given by 2 * arctan(0.033/r) where
  r is the distance to the center of the can.

### Reference: Minimal ROS2 subscriber code

```cpp
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

### Reference: Minimal ROS2 publisher code

```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

## Low-level tasks

These tasks are ordered from start to finish

1. Create ScanAnalyser class.

```aider
Create 'src/ScanAnalyser.cpp'
Define the class ScanAnalyzer
    MIRROR the provided Minimal Subscriber code to create the class ScanAnalyzer
        subscribe to the topic '/scan/unfiltered' with message type sensor_msgs/msg/LaserScan
    MIRROR the provided Minimal Publisher code to add a publisher to the ScanAnalyzer class.
          It will publish to the
          topic '/can_scan' with message type sensor_msgs/msg/LaserScan
    MIRROR the provided Minimal Publisher code to add a third publisher to the ScanAnalyzer class.
          It will publish to the topic '/goal_can_pos' with message type geometry_msgs/msg/PointStamped
```

2. Rotate the data in the scan message 180 degrees

The data in the scan message represents lidar data from 360 degrees. Cans of interest can
be directly in front of the scanner, and the lidar data from them can begin at the end of the
message and span the end to the beginning of the message and end near the beginning of
the message. By rotating the data in the buffer, we ensure that cans directly in front of
the scanner is contiguous, in the middle of the buffer.

```aider
    In the subscriber callback, rotate the received scan message half the buffer length
          by copying the last half of the
          input scan to the beginning of a sensor_msgs/msg/LaserScan message called 'scan_can_rot'
          and then copy the first half of the input
          scan to the rest of the 'can_scan_rot' message.
          The 'can_scan_rot' message should look as if the scan started half way
          through the original input.
    Copy the contents of the 'can_scan_rot' message to a new sensor_msgs/msg/LaserScan message
          called 'found_cans_rot' 
    Call the 'findCans' method described below, passing it references to the 'can_scan_rot'
          and 'found_cans_rot' messages. It will modify
          the 'found_cans_rot' message, but not the 'can_scan_rot' message
```

3. Add 'findCans' method to identify sequences of scan ranges that represent soda cans

```aider
    - findCans shall search the list of ranges in the 'scan_can_rot' message and
    identify valid sequences of ranges that represent a can.
    - findCans should overwrite each range value in the 'found_cans_rot' message with NaN except for those
    that are part of a valid sequence that represents a found can
    - In findCans, sequence validation should not stop with any NaNs. Valid sequences may contain NaNs
    - Validation rules:
      - The range value difference between successive points must be less than 0.02
      - Range must be less than 1.0
      - The number of range values that are not NaN in a valid sequence must be >= 2 * arctan(0.015/r)
      and <= 2*arctan(0.04/r) where r is the range of the middle entry in the sequence.
      - The range to the midpoint of the sequence must be less than the range to either end of the sequence
```

4. Publish the '/can_scan' message

In this step we rotate the data representing cans 180 degrees to restore it to the orientation of
the input scan data then we publish it.

```aider
    After 'findCans' has processed the rotated input scan message and modified 'found_cans_rot',
    rotate the found_cans_rot message half the buffer length by copying the last half of the buffer
    to the beginning of the '/can_scan' sensor_msgs/msg/LaserScan
    message and then copy the first half of the buffer to the rest of the '/can_scan' message.
    The '/can_scan' message should look as if it started half way through the found_cans_rot message.
    Publish '/can_scan'
```

5. Find the closest can and publish its position

```aider
  Look at all the valid sequences found and find the one with the minimum range to its midpoint. This is the closest can.
  Compute the angle to the midpoint of the closest can's sequence by using the midpoint's index in the message as follows:
  - theta = midpoint-index * angle_increment
  Next, compute the X and Y coordinates of the midpoint relative to the lidar as follows:
  - X = range * sin(theta)
  - Y = range * cos(theta)
  Create a geometry_msgs/msg/PointStamped message and fill its header fields from the '/can_scan' message and
  fill X and Y with the coordinates of the midpoint.

```

6. Update build files

```aider
/add CMakeLists.txt
/add package.xml
update the ROS2 build control files to create the scan_analyzer app from the ScanAnalyser.cpp file
```

