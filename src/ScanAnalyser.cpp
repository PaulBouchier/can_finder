#include <functional>
#include <memory>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm> // Required for std::copy

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point_stamped.hpp" // Added for PointStamped message

using std::placeholders::_1;

/**
 * @brief Node that subscribes to LaserScan data, finds cans, and publishes modified scans.
 */
class ScanAnalyser : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Scan Analyser object
   */
  ScanAnalyser()
  : Node("scan_analyser") // Node name
  {
    // Subscriber to the unfiltered laser scan topic
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan/unfiltered", 10, std::bind(&ScanAnalyser::scan_callback, this, _1));

    // Publisher for the rotated laser scan
    rot_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/can_scan_rot", 10);

    // Publisher for the laser scan showing only cans
    can_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/can_scan", 10);

    // Publisher for the closest can position
    point_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/goal_can_pos", 10); // Added publisher

    // Publisher for the original scan (for comparison/debugging)
    scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    RCLCPP_INFO(this->get_logger(), "ScanAnalyser node started.");

    // Initialize closest can tracking variables
    closest_can_found_ = false; // Added initialization
  }

private:
  /**
   * @brief Callback function for the laser scan subscriber.
   * Processes the incoming scan, rotates it, finds cans, and publishes results.
   * @param msg The received LaserScan message.
   */
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Publish the original incoming scan
    scan_publisher_->publish(*msg);

    // Reset closest can info for this scan
    closest_can_found_ = false;

    // Create messages for publishing
    auto rotated_scan = std::make_unique<sensor_msgs::msg::LaserScan>();
    auto can_only_scan = std::make_unique<sensor_msgs::msg::LaserScan>();

    // Copy header and metadata from the input message
    rotated_scan->header = msg->header;
    rotated_scan->angle_min = msg->angle_min;
    rotated_scan->angle_max = msg->angle_max;
    rotated_scan->angle_increment = msg->angle_increment;
    rotated_scan->time_increment = msg->time_increment;
    rotated_scan->scan_time = msg->scan_time;
    rotated_scan->range_min = msg->range_min;
    rotated_scan->range_max = msg->range_max;
    rotated_scan->ranges.resize(msg->ranges.size());
    rotated_scan->intensities = msg->intensities; // Copy intensities if they exist

    can_only_scan->header = msg->header;
    can_only_scan->angle_min = msg->angle_min;
    can_only_scan->angle_max = msg->angle_max;
    can_only_scan->angle_increment = msg->angle_increment;
    can_only_scan->time_increment = msg->time_increment;
    can_only_scan->scan_time = msg->scan_time;
    can_only_scan->range_min = msg->range_min;
    can_only_scan->range_max = msg->range_max;
    can_only_scan->ranges.resize(msg->ranges.size());
    can_only_scan->intensities = msg->intensities; // Copy intensities if they exist

    // Rotate the scan data by half the buffer length
    size_t num_ranges = msg->ranges.size();
    if (num_ranges == 0) {
        RCLCPP_WARN(this->get_logger(), "Received empty scan message.");
        return; // Nothing to process
    }
    size_t half_size = num_ranges / 2;
    size_t first_half_size = num_ranges - half_size; // Handle odd sizes

    // Copy last half of input scan to the beginning of rotated_scan
    std::copy(msg->ranges.begin() + first_half_size, msg->ranges.end(), rotated_scan->ranges.begin());
    // Copy first half of input scan to the end of rotated_scan
    std::copy(msg->ranges.begin(), msg->ranges.begin() + first_half_size, rotated_scan->ranges.begin() + half_size);

    // Copy the rotated scan to the can_only_scan (will be modified by findCans)
    can_only_scan->ranges = rotated_scan->ranges;

    // Find cans in the rotated scan and modify can_only_scan
    findCans(*rotated_scan, *can_only_scan);

    // Publish the rotated scan (intermediate result)
    rot_publisher_->publish(std::move(rotated_scan));

    // --- Rotate can_only_scan back to original orientation ---
    auto final_can_scan = std::make_unique<sensor_msgs::msg::LaserScan>();

    // Copy header and metadata from the processed can_only_scan
    final_can_scan->header = can_only_scan->header; // Use the latest timestamp etc.
    final_can_scan->angle_min = can_only_scan->angle_min;
    final_can_scan->angle_max = can_only_scan->angle_max;
    final_can_scan->angle_increment = can_only_scan->angle_increment;
    final_can_scan->time_increment = can_only_scan->time_increment;
    final_can_scan->scan_time = can_only_scan->scan_time;
    final_can_scan->range_min = can_only_scan->range_min;
    final_can_scan->range_max = can_only_scan->range_max;
    final_can_scan->ranges.resize(can_only_scan->ranges.size());
    final_can_scan->intensities = can_only_scan->intensities; // Copy intensities

    // Perform the inverse rotation if there are ranges
    if (num_ranges > 0) {
        // Copy the second half of can_only_scan (originally the first half of msg)
        // to the beginning of final_can_scan
        std::copy(can_only_scan->ranges.begin() + half_size, can_only_scan->ranges.end(),
                  final_can_scan->ranges.begin());
        // Copy the first half of can_only_scan (originally the second half of msg)
        // to the end of final_can_scan
        std::copy(can_only_scan->ranges.begin(), can_only_scan->ranges.begin() + half_size,
                  final_can_scan->ranges.begin() + first_half_size);
    }
    // --- End Rotation Back ---

    // Publish the scan containing only cans, rotated back to original orientation
    can_publisher_->publish(std::move(final_can_scan));

    // --- Publish Closest Can Position ---
    if (closest_can_found_) {
      // Calculate the original index before rotation
      size_t original_index;
      // Need half_size and first_half_size from earlier in the function
      size_t num_ranges = msg->ranges.size(); // Recalculate or pass as argument if needed
      size_t half_size = num_ranges / 2;
      size_t first_half_size = num_ranges - half_size;

      if (closest_can_index_ < half_size) { // Index was in the second half of original scan (now first half of rotated)
        original_index = closest_can_index_ + first_half_size;
      } else { // Index was in the first half of original scan (now second half of rotated)
        original_index = closest_can_index_ - half_size;
      }

      // Calculate the angle corresponding to the original index
      double theta_actual = msg->angle_min + original_index * msg->angle_increment;

      // Calculate X and Y coordinates (ROS REP 103: X forward, Y left)
      double x = closest_can_range_ * std::cos(theta_actual);
      double y = closest_can_range_ * std::sin(theta_actual);

      // Create and populate the PointStamped message
      auto point_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
      point_msg->header.stamp = this->get_clock()->now(); // Use current time or msg->header.stamp
      point_msg->header.frame_id = msg->header.frame_id; // Use frame_id from input scan
      point_msg->point.x = x;
      point_msg->point.y = y;
      point_msg->point.z = 0.0; // Assuming 2D lidar
      point_publisher_->publish(std::move(point_msg));
    }
  }

  /**
   * @brief Finds sequences of laser scan points that represent cans.
   * Modifies the can_only_scan message to contain only points identified as cans.
   * @param rotated_scan The input laser scan data (rotated).
   * @param can_only_scan The output laser scan data, initially a copy of rotated_scan,
   *                      modified to contain NaN for non-can points.
   */
  void findCans(const sensor_msgs::msg::LaserScan& rotated_scan, sensor_msgs::msg::LaserScan& can_only_scan)
  {
    const float MAX_RANGE = 1.0; // Maximum range to consider for a can
    const float MAX_DIFF = 0.02; // Maximum range difference between consecutive points
    const float CAN_RADIUS_MIN = 0.015; // Minimum radius used for angle calculation
    const float CAN_RADIUS_MAX = 0.04;  // Maximum radius used for angle calculation
    const float MIN_RANGE_FOR_ARC = 0.01; // Minimum range to avoid division by zero/instability

    size_t num_ranges = rotated_scan.ranges.size();
    float angle_increment = rotated_scan.angle_increment;
    float nan_value = std::numeric_limits<float>::quiet_NaN();

    // Initialize can_only_scan ranges to NaN
    std::fill(can_only_scan.ranges.begin(), can_only_scan.ranges.end(), nan_value);

    // Variables to track the closest can found in this scan
    float min_mid_range = std::numeric_limits<float>::infinity();
    size_t best_mid_index = 0; // Index in rotated_scan
    bool found_any_can = false;

    // Iterate through the scan points to find potential can sequences
    for (size_t i = 0; i < num_ranges; ) {
      float current_range = rotated_scan.ranges[i];

      // Potential start of a sequence: not NaN and within max range
      if (!std::isnan(current_range) && current_range < MAX_RANGE && current_range >= rotated_scan.range_min) {
        size_t start_index = i;
        size_t end_index = i;
        int non_nan_count = 1;
        float last_valid_range = current_range;
        std::vector<float> sequence_ranges; // Store non-NaN ranges for midpoint calculation
        sequence_ranges.push_back(current_range);


        // Check subsequent points for sequence continuation
        for (size_t j = i + 1; j < num_ranges; ++j) {
          float next_range = rotated_scan.ranges[j];

          if (!std::isnan(next_range)) {
            // Check validation rules for non-NaN points
            if (next_range >= MAX_RANGE || next_range < rotated_scan.range_min || std::abs(next_range - last_valid_range) >= MAX_DIFF) {
              // Sequence broken by this point
              break;
            } else {
              // Valid point in sequence
              end_index = j;
              non_nan_count++;
              last_valid_range = next_range;
              sequence_ranges.push_back(next_range);
            }
          } else {
            // NaN point - potentially continue sequence, but don't update last_valid_range or count
             end_index = j; // Extend sequence over NaN
          }
        } // End inner loop (sequence checking)

        // --- Sequence Validation ---
        bool sequence_is_can = false;
        if (non_nan_count >= 2) { // Need at least 2 valid points
            // Find the range 'r' of the middle non-NaN point in the sequence
            float r = nan_value;
            size_t middle_seq_index = sequence_ranges.size() / 2; // Index within sequence_ranges
            if (middle_seq_index < sequence_ranges.size()) {
                r = sequence_ranges[middle_seq_index];
            }


            if (!std::isnan(r) && r > MIN_RANGE_FOR_ARC) {
                // Check if midpoint range is less than start and end ranges
                float first_range = sequence_ranges.front();
                float last_range = sequence_ranges.back();
                bool midpoint_closer = (r < first_range) && (r < last_range);

                // Calculate expected angle range based on can diameter and range 'r'
                float min_angle = 2.0 * std::atan(CAN_RADIUS_MIN / r);
                float max_angle = 2.0 * std::atan(CAN_RADIUS_MAX / r);

                // Convert angles to number of points/indices
                // Add 1 because N points span N-1 increments, but we count points
                int min_points_needed = static_cast<int>(std::ceil(min_angle / angle_increment));
                int max_points_allowed = static_cast<int>(std::floor(max_angle / angle_increment)) +1; // Allow slightly wider angle

                // Check if the number of non-NaN points falls within the calculated bounds
                // AND check if the midpoint is closer than the ends
                if (non_nan_count >= min_points_needed && non_nan_count <= max_points_allowed && midpoint_closer) {
                    sequence_is_can = true;

                    // --- Check if this is the closest can found so far ---
                    if (r < min_mid_range) {
                        min_mid_range = r;
                        found_any_can = true;

                        // Find the actual index in rotated_scan corresponding to the midpoint range 'r'
                        // Iterate through the original sequence indices (including NaNs)
                        // and count non-NaNs until we reach the middle one.
                        int current_non_nan_count = 0;
                        for(size_t k = start_index; k <= end_index; ++k) {
                            if (!std::isnan(rotated_scan.ranges[k])) {
                                if (current_non_nan_count == static_cast<int>(middle_seq_index)) {
                                    best_mid_index = k; // Store the index in rotated_scan
                                    break;
                                }
                                current_non_nan_count++;
                            }
                        }
                    }
                }
            }
        }

        // If sequence is validated as a can, copy its points to can_only_scan
        if (sequence_is_can) {
          for (size_t k = start_index; k <= end_index; ++k) {
             // Only copy original non-NaN values that were part of the sequence logic
             // This prevents copying NaNs that were just bridged over
             if (!std::isnan(rotated_scan.ranges[k])) {
                 can_only_scan.ranges[k] = rotated_scan.ranges[k];
             }
          }
           // Log found can info (optional)
           // RCLCPP_INFO(this->get_logger(), "Found can: indices %zu-%zu, points: %d", start_index, end_index, non_nan_count);
        }

        // Move outer loop index past the processed sequence
        i = end_index + 1;

      } else {
        // Current point is not a valid start, move to the next point
        i++;
      }
    } // End outer loop (scan iteration)

   // After checking all sequences, update the class members if a can was found
   if (found_any_can) {
       this->closest_can_found_ = true;
       this->closest_can_range_ = min_mid_range;
       this->closest_can_index_ = best_mid_index; // Index relative to rotated_scan
   }
  }


  // Member variables
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr rot_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr can_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_publisher_; // Added point publisher
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_; // Publisher for original scan
  bool closest_can_found_;        // Flag if a can was found in the last scan
  float closest_can_range_;       // Range to the midpoint of the closest can
  size_t closest_can_index_;      // Index (in rotated_scan) of the midpoint of the closest can
};

/**
 * @brief Main function for the ScanAnalyser node.
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Create and spin the ScanAnalyser node
  rclcpp::spin(std::make_shared<ScanAnalyser>());
  rclcpp::shutdown();
  return 0;
}
