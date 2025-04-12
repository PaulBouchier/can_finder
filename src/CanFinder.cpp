#include <functional>
#include <memory>
#include <vector>
#include <cmath>
#include <limits> // Required for std::numeric_limits

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "example_interfaces/srv/set_bool.hpp" // Service type

// Use placeholders for std::bind
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3; // Needed for service handler

// Define message types for convenience
using LaserScan = sensor_msgs::msg::LaserScan;
using PoseArray = geometry_msgs::msg::PoseArray;
using SetBool = example_interfaces::srv::SetBool;

/**
 * @brief The CanFinder class subscribes to laser scan data, finds cans,
 *        and publishes their positions and a filtered scan.
 */
class CanFinder : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Can Finder object
   */
  CanFinder()
  : Node("can_finder"), blankFwdSector(false) // Initialize blankFwdSector to false
  {
    // Subscriber to the unfiltered laser scan data
    subscription_ = this->create_subscription<LaserScan>(
      "/scan/unfiltered", 10, std::bind(&CanFinder::scanCb, this, _1));

    // Publisher for the potentially modified scan data (e.g., blanked sector)
    scan_publisher_ = this->create_publisher<LaserScan>("/scan", 10);

    // Publisher for the laser scan data containing only points on detected cans
    can_scan_publisher_ = this->create_publisher<LaserScan>("/can_scan", 10);

    // Publisher for the positions of detected cans
    can_positions_publisher_ = this->create_publisher<PoseArray>("/can_positions", 10);

    // Service server to control blanking the forward sector
    blank_fwd_sector_service_ = this->create_service<SetBool>(
        "/blank_fwd_sector",
        std::bind(&CanFinder::blankFwdSectorCb, this, _1, _2, _3));

    RCLCPP_INFO(this->get_logger(), "CanFinder node initialized.");
  }

private:
  /**
   * @brief Callback function for the laser scan subscriber.
   * @param msg The received LaserScan message.
   */
  void scanCb(const LaserScan::SharedPtr msg) const
  {
    // Implementation will be added in the next step (Task 2)
    RCLCPP_DEBUG(this->get_logger(), "Received scan with %zu ranges.", msg->ranges.size());
    // Placeholder: Just log receipt for now
  }

  /**
   * @brief Service handler callback for the /blank_fwd_sector service.
   * @param request_header The request header (unused).
   * @param request The service request containing the boolean data.
   * @param response The service response (unused, always indicates success).
   */
  void blankFwdSectorCb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<SetBool::Request> request,
    const std::shared_ptr<SetBool::Response> response)
  {
    (void)request_header; // Indicate unused parameter
    RCLCPP_INFO(this->get_logger(), "Received request to set blankFwdSector to: %s", request->data ? "true" : "false");
    blankFwdSector = request->data;
    response->success = true;
    // response->message is optional, leave empty for now
  }

  // Private methods to be implemented later
  void blankCapturedCan(LaserScan & scan) const {}
  void rotateScan(const LaserScan & scan_in, LaserScan & scan_out) const {}
  void findCans(const LaserScan & scan_in, LaserScan & can_scan_out, PoseArray & found_cans_out) const {}


  // Member Variables
  rclcpp::Subscription<LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<LaserScan>::SharedPtr scan_publisher_;
  rclcpp::Publisher<LaserScan>::SharedPtr can_scan_publisher_;
  rclcpp::Publisher<PoseArray>::SharedPtr can_positions_publisher_;
  rclcpp::Service<SetBool>::SharedPtr blank_fwd_sector_service_;
  bool blankFwdSector; // Flag to control blanking
};

/**
 * @brief Main function to initialize and run the CanFinder node.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return Exit code.
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Create and spin the CanFinder node
  rclcpp::spin(std::make_shared<CanFinder>());
  rclcpp::shutdown();
  return 0;
}
