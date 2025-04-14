# ROS2 Jazzy Node: Find cans in lidar scandata

Ingest the information from this file, implement the low-level tasks, and generate
the code that will satisfy the high and mid-level objectives

## High-level Objective

Add a laser scan data analysis node to the directory structure. The node is to find
soda cans in the incoming laser scan data.

## Mid-level Objective

The node is to be a ROS2 publisher and subscriber. It shall analyze the
laser scan data and identify the soda cans it finds in the laser scan
data and output the positions of the found cans relative to the laser scanner. It shall
also publish a sensor_msgs/msg/LaserScan message to topic '/can_scan' that contains only
points which are on a can.
The node will identify which points are on a can using a filter described later.

## Implementation Notes
- Be sure to follow the low-level tasks in order and in detail
- Use the current git branch
- This will be a C++ node
- add DocStrings;
- comment the code;

## Context

### Previously executed commands

```bash
cd ~/ros2_ws/src
# create a new package for the can_finder
ros2 pkg create --build-type ament_cmake --license Apache-2.0 --node-name can_finder can_finder
# Build the workspace
cd ~/ros2_ws
colcon build --symlink-install
```

### Beginning context

/add src/CanFinder.cpp
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

### Reference: Minimal Service Server code

```cpp
#include <cinttypes>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;
rclcpp::Node::SharedPtr g_node = nullptr;

void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<AddTwoInts::Request> request,
  const std::shared_ptr<AddTwoInts::Response> response)
{
  (void)request_header;
  RCLCPP_INFO(
    g_node->get_logger(),
    "request: %" PRId64 " + %" PRId64, request->a, request->b);
  response->sum = request->a + request->b;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("minimal_service");
  auto server = g_node->create_service<AddTwoInts>("add_two_ints", handle_service);
  rclcpp::spin(g_node);
  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}
```

## Low-level tasks

These tasks are ordered from start to finish

1. Create CanFinder class.

```aider
  Create 'src/CanFinder.cpp'
  Define the class CanFinder:
    - MIRROR the provided Minimal Subscriber code to create the class CanFinder
      subscribe to the topic '/scan/unfiltered' with message type sensor_msgs/msg/LaserScan.
      Add the private callback method called 'scanCb' which is defined below
    - MIRROR the provided Minimal Publisher code to add a publisher to the CanFinder class.
      It will publish to the topic '/scan' with message type sensor_msgs/msg/LaserScan
    - MIRROR the provided Minimal Publisher code to add a publisher to the CanFinder class.
      It will publish to the topic '/can_scan' with message type sensor_msgs/msg/LaserScan
    - MIRROR the provided Minimal Publisher code to add a publisher to the CanFinder class.
      It will publish to the topic '/can_positions' with message type geometry_msgs/msg/PoseArray
    - MIRROR the provided Minimal Publisher code to add a publisher to the CanFinder class.
      It will publish to the topic '/closest_range_bearing' with message type geometry_msgs/msg/Point
    - Add the private boolean variable 'blankFwdSector'
    - MIRROR the provided Minimal Service Server code to add a service server to the CanFinder class.
      It will provide the service '/blank_fwd_sector' with type 'example_interfaces/srv/SetBool'
      and will copy 'data' to 'blank_fwd_sector' and always return success.
```

2. 'scanCb' method definition

```aider
  In the subscriber callback, perform the following:
    - If 'blank_fwd_sector is true, call a private method called 'blankCapturedCan'.
      Pass 'blankCapturedCan' a reference to the received scan message, (which it may modify).
      Then publish the received scan message to the '/scan' topic.
    - Call a private method called 'rotateScan' which is defined below. Pass
      it a reference to the received scan message, and a reference to a new
      sensor_msgs/msg/LaserScan message called 'rotatedScan' that is its output.
    - Call a private method called 'findCans', which is defined below. Pass it
      a reference to 'rotatedScan', and a reference to a new
      sensor_msgs/msg/LaserScan message called 'canScanRot' and a reference to
      a new geometry_msgs/msg/PoseArray message called 'foundCans' which are its
      outputs
    - Call 'rotateScan' and pass it 'canScanRot' as input and a reference to a new
        sensor_msgs/msg/LaserScan message called 'canScan' which is its output
    - Publish 'canScan' to the '/can_scan' topic
    - Call the 'transformCanPoses' method defined below, passing a reference to the foundCans
    message. 'transformCanPoses' will modify each can pose by transforming it from the
    laser frame to the map frame.
    - Publish the foundCans message to the '/can_positions' topic
```

3. 'blankCapturedCan' method definition

    Blank the output lidar data that is from a can that is in the jaws and being transported, in
          order that nav2 navigation won't detect a collision

```aider
    Replace the range data in the referenced input ranges field at index 0 to 25 with NaN and at index 430
      to the end with NaN
```

4.  'rotateScan' method definition

  The data in the input scan message represents lidar data from a 360 degrees scan. Cans of interest can
  be directly in front of the scanner, and the lidar data from them can begin near the end of the
  message and span the end to the beginning of the message and end near the beginning of
  the message. By rotating the data in the buffer 180 degrees, we ensure that sequences of ranges from
  cans directly in front of the scanner are contiguous, in the middle of the buffer.

  The output message should look as if the scan started half way through the original input.
  Calling 'rotateScan' a second time unrotates the data.

```aider
  - Copy all the fields except ranges and intensities from the input to the output
  - Copy the last half of the ranges array data in the input scan message to the beginning of ranges array
    in the provided output scan message. Then copy the first half of the ranges array to the rest of
    the output message.
```

5. 'findCans' method definition

  Identify sequences of scan ranges that represent soda cans
  by searching the list of ranges in the input message and applying validation rules.
  Output these sequences along with an array of their positions.
  Note that in the lidar frame, x faces forward and y to the left

```aider
  - Copy the input scan message to the output scan message
  - Overwrite each range value in the output message with NaN except for those
    that are part of a valid sequence that represents a found can
  - Validation rules:
    - Sequence validation should not stop with any NaNs. Valid sequences may contain NaNs
    - The range value difference between successive points must be less than 0.02
    - Range must be less than 1.0
    - The number of range values that are not NaN in a valid sequence must be >= (2 * arctan(0.015/r) / angle_increment)
    and <= (2*arctan(0.04/r) / angle_increment) where r is the range of the middle entry in the sequence.
    - The range to the midpoint of the sequence must be less than the range to either end of the sequence
  - Fill the header fields of the provided PoseArray message from the input scan message header.
  - Sort the valid sequences in order of minimum range to the midpoint
  - For each valid sequence, add an element to the poses array representing the x and y positions
  of the sequence midpoint as follows:
    - Compute the angle from the beginning of the scan to the midpoint of
    the sequence by using the midpoint's range index in the message as follows:
      - theta = ((midpoint-index * angle_increment) + pi)
      - Next, compute the X and Y coordinates of the midpoint relative to the lidar as follows:
        - position.y = (range * sin(theta))
        - position.x = (range * cos(theta))
        - fill position.z and the orientation fields with 0.0
  - For the closest sequence only, populate a geometry_msgs/msg/Point message called 'range_bearing' as follows:
    - x = midpoint range
    - y = theta
  - Publish the 'range_bearing' message to the '/closest_range_bearing' topic 
```


6. 'transformCanPoses' method definition

Convert found_cans pose array from laser frame to map frame.

```aider
  - Change the frame_id of the PoseArray message to 'map'
  - Transform each pose in the PoseArray message to the map frame as shown in the
  Reference Transform Listener code example and overwrite the input pose with the transformed
  pose.
```

7. Update build files

```aider
/add CMakeLists.txt
/add package.xml
update the ROS2 build control files to create the can_finder app from the CanFinder.cpp file
```

