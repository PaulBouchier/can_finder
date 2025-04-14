#include <functional>
#include <memory>
#include <vector>
#include <cmath>
#include <algorithm> // For std::copy, std::sort, std::fill
#include <limits>    // Required for std::numeric_limits

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "example_interfaces/srv/set_bool.hpp" // Service type
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // For geometry_msgs transforms
#include "geometry_msgs/msg/pose_stamped.hpp"     // For PoseStamped
#include "geometry_msgs/msg/point.hpp"            // For Point message
#include "tf2/exceptions.h"                       // For transform exceptions
#include <string>                                  // For std::string

// Use placeholders for std::bind
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3; // Needed for service handler

// Define message types for convenience
using LaserScan = sensor_msgs::msg::LaserScan;
using PoseArray = geometry_msgs::msg::PoseArray;
using SetBool = example_interfaces::srv::SetBool;

/**
 * @brief Represents a potential can sequence found in the laser scan data.
 */
struct CanSequence {
    size_t start_index;
    size_t end_index;
    size_t mid_index; // Index of the point used for mid_range
    float mid_range;
    size_t valid_points_count; // Number of non-NaN points in the sequence

    // For sorting by range
    bool operator<(const CanSequence& other) const {
        return mid_range < other.mid_range;
    }
};


/**
 * @brief Transforms can poses into a target frame using TF2.
 */
class CanPoseTransformer
{
public:
  /**
   * @brief Construct a new Can Pose Transformer object.
   * @param node A pointer to the parent node for logging.
   * @param tf_buffer A shared pointer to the TF2 buffer.
   */
  CanPoseTransformer(rclcpp::Node* node, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
    : node_(node), tf_buffer_(tf_buffer)
  {
    if (!node_) {
      throw std::invalid_argument("Node pointer cannot be null in CanPoseTransformer constructor");
    }
    if (!tf_buffer_) {
      throw std::invalid_argument("TF buffer pointer cannot be null in CanPoseTransformer constructor");
    }
    RCLCPP_INFO(node_->get_logger(), "CanPoseTransformer initialized.");
  }

  /**
   * @brief Transforms the poses within a PoseArray message to the target frame.
   * Modifies the input PoseArray message in place.
   * @param poses The PoseArray message containing poses to transform.
   * @param target_frame The desired target frame ID (e.g., "map").
   */
  void transformPoses(geometry_msgs::msg::PoseArray& poses, const std::string& target_frame)
  {
    if (poses.poses.empty()) {
      // No poses to transform, just update header if needed (though unlikely needed if empty)
      // poses.header.frame_id = target_frame; // Optional: update frame even if empty
      return;
    }

    const std::string source_frame = poses.header.frame_id;
    if (source_frame.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Cannot transform poses: Source frame ID is empty.");
        return;
    }
    if (source_frame == target_frame) {
        RCLCPP_DEBUG(node_->get_logger(), "Source and target frames are the same ('%s'), skipping transformation.", target_frame.c_str());
        return;
    }

    geometry_msgs::msg::PoseStamped pose_stamped_in, pose_stamped_out;
    pose_stamped_in.header = poses.header; // Use header from PoseArray

    for (auto& pose : poses.poses) {
      pose_stamped_in.pose = pose; // Set the current pose

      try {
        // Use tf2::TimePointZero for latest available transform
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(target_frame,
                                   pose_stamped_in.header.frame_id, tf2::TimePointZero);
     
        tf2::doTransform(pose_stamped_in, pose_stamped_out, transform);

        // Update the pose in the array with the transformed one
        pose = pose_stamped_out.pose;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(node_->get_logger(), "Could not transform pose from %s to %s: %s",
                     source_frame.c_str(), target_frame.c_str(), ex.what());
        // Decide how to handle failed transforms. Options:
        // 1. Skip this pose (leave it untransformed - might be confusing)
        // 2. Remove this pose from the array (requires careful index handling or post-processing)
        // 3. Clear the whole array and return (simplest if any failure is critical)

        // For now, let's log the error and leave the pose untransformed in the array.
        // Consider adding logic to remove failed poses if required.
        continue; // Move to the next pose
      }
    }

    // Update the header frame_id to the target frame AFTER transforming all poses
    poses.header.frame_id = target_frame;
    // Timestamp is usually kept from the original message or updated just before publishing
    // poses.header.stamp = node_->get_clock()->now(); // Optional: update stamp here too
  }

private:
  rclcpp::Node* node_; // Pointer to the parent node for logging
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

/**
 * @brief The CanFinder class subscribes to laser scan data, finds cans,
 *        and publishes their positions and a filtered scan.
 */
class CanFinder : public rclcpp::Node
{
private:
  // Define constants
  static constexpr double CAN_DIAMETER = 0.066; // meters
  static constexpr double CAN_RADIUS = CAN_DIAMETER / 2.0;
  static constexpr double MIN_CAN_RADIUS_DETECT = 0.015; // meters, for validation rule
  static constexpr double MAX_CAN_RADIUS_DETECT = 0.04;  // meters, for validation rule
  static constexpr double MAX_RANGE_DIFF = 0.02;         // meters, for sequence validation
  static constexpr double MAX_CAN_RANGE = 1.0;           // meters, for sequence validation
  static constexpr double PI = M_PI; // Use M_PI from cmath

public:
  /**
   * @brief Construct a new Can Finder object
   */
  CanFinder()
  : Node("can_finder"), blankFwdSector_(false) // Initialize blankFwdSector_
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
        // Use only _1 and _2 placeholders
        std::bind(&CanFinder::blankFwdSectorCb, this, _1, _2));

    // Initialize TF2 buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Instantiate the CanPoseTransformer
    can_pose_transformer_ = std::make_unique<CanPoseTransformer>(this, tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "CanFinder node initialized.");
  }

private:
  /**
   * @brief Callback function for the laser scan subscriber.
   * @param msg The received LaserScan message.
   */
  void scanCb(const LaserScan::SharedPtr msg) // Keep SharedPtr, make copy inside
  {
    // Create a mutable copy for potential modification by blankCapturedCan
    LaserScan current_scan = *msg; // Dereference SharedPtr to copy

    // 1. Blank forward sector if requested
    if (blankFwdSector_) {
        blankCapturedCan(current_scan); // Pass the mutable copy
        // Publish the potentially modified scan to /scan
        scan_publisher_->publish(current_scan);
    } else {
        // If not blanking, publish the original scan (or the copy) to /scan
        scan_publisher_->publish(current_scan); // Publish the copy
    }

    // 2. Rotate scan data for easier processing
    LaserScan rotatedScan;
    // Pass the mutable copy, remove const from current_scan if rotateScan needs non-const (it does now)
    rotateScan(current_scan, rotatedScan);

    // 3. Find cans in the rotated scan
    LaserScan canScanRot;
    PoseArray foundCans;
    std::vector<CanSequence> potential_cans; // Declare vector here
    // Pass rotatedScan (non-const) and potential_cans
    findCans(rotatedScan, canScanRot, foundCans, potential_cans);

    // 4. Rotate the can-only scan back to the original orientation
    LaserScan canScan;
    rotateScan(canScanRot, canScan); // Rotate back using canScanRot

    // 5. Publish the can-only scan
    // Ensure the header timestamp is current
    canScan.header.stamp = this->get_clock()->now();
    can_scan_publisher_->publish(canScan);

    // 5b. Publish range and bearing of the closest can (if any)
    if (!potential_cans.empty()) { // Check if any potential cans were found and validated
        // The closest can is the first one after sorting by range
        const auto& closest_can = potential_cans[0];
        float closest_range = closest_can.mid_range;
        size_t closest_mid_index = closest_can.mid_index;
        // Use angle_increment from the scan passed *into* findCans
        float angle_increment = rotatedScan.angle_increment;

        // Calculate bearing relative to the original lidar forward direction (X-axis)
        // The angle calculation in findCans already accounts for the rotation:
        // theta = (mid_index * angle_increment) + PI is the angle in the original frame.
        double theta = (static_cast<double>(closest_mid_index) * angle_increment) + PI;
        // Normalize bearing (theta) to [-pi, pi]
        theta = atan2(sin(theta), cos(theta));

        Point range_bearing; // Renamed from closest_point_msg
        range_bearing.x = static_cast<double>(closest_range); // Range
        range_bearing.y = theta; // Bearing (angle)
        range_bearing.z = 0.0; // Set z to 0.0 as per request

        closest_can_publisher_->publish(range_bearing); // Publish the message
    }

    // 6. Publish the positions of the found cans
    // 6a. Transform can poses to the "map" frame
    if (can_pose_transformer_) { // Check if transformer was initialized successfully
        can_pose_transformer_->transformPoses(foundCans, "map");
        // The transformPoses method updates foundCans.header.frame_id internally
    } else {
        RCLCPP_WARN(this->get_logger(), "CanPoseTransformer not initialized. Cannot transform poses.");
        // Keep original frame_id if transformer failed
        foundCans.header.frame_id = msg->header.frame_id;
    }

    // 6b. Publish the (potentially transformed) positions of the found cans
    // Ensure the header timestamp is current (transformPoses might not update it)
    foundCans.header.stamp = this->get_clock()->now();
    // Frame ID is now set by transformPoses or kept original if transform failed
    can_positions_publisher_->publish(foundCans);
  }

  /**
   * @brief Service handler callback for the /blank_fwd_sector service.
   * @param request_header The request header (unused).
   * @param request The service request containing the boolean data.
   * @param response The service response.
   */
  void blankFwdSectorCb(
    // Remove request_header parameter
    const std::shared_ptr<SetBool::Request> request,
    std::shared_ptr<SetBool::Response> response)
  {
    // (void)request_header; // Remove this line
    RCLCPP_INFO(this->get_logger(), "Received request to set blankFwdSector to: %s", request->data ? "true" : "false");
    // Use member variable directly (ensure it's mutable)
    blankFwdSector_ = request->data; // Assuming member is named blankFwdSector_
    response->success = true;
    response->message = blankFwdSector_ ? "Forward sector blanking enabled" : "Forward sector blanking disabled";
  }

  /**
   * @brief Blanks out the forward sector of the laser scan ranges.
   * Modifies the input scan message directly.
   * @param scan The LaserScan message to modify.
   */
  void blankCapturedCan(LaserScan & scan) // Remove const
  {
      // Indices to blank: 0 to 25 and 430 to end
      size_t num_ranges = scan.ranges.size();
      if (num_ranges == 0) return; // No data to blank

      float nan_value = std::numeric_limits<float>::quiet_NaN();

      // Blank 0 to 25
      size_t end_first_blank = std::min((size_t)25, num_ranges - 1);
      for (size_t i = 0; i <= end_first_blank; ++i) {
          scan.ranges[i] = nan_value;
      }

      // Blank 430 to end
      if (num_ranges > 430) {
          for (size_t i = 430; i < num_ranges; ++i) {
              scan.ranges[i] = nan_value;
          }
      }

      // Also blank intensities if they exist and match ranges size
      if (scan.intensities.size() == num_ranges) {
           for (size_t i = 0; i <= end_first_blank; ++i) {
              scan.intensities[i] = nan_value;
          }
          if (num_ranges > 430) {
              for (size_t i = 430; i < num_ranges; ++i) {
                  scan.intensities[i] = nan_value;
              }
          }
      }
  }

  /**
   * @brief Rotates the laser scan data by 180 degrees (pi radians).
   * This places the front-facing sector (around angle 0) in the middle of the arrays.
   * @param scan_in The original LaserScan message.
   * @param scan_out The LaserScan message to store the rotated data.
   */
  void rotateScan(LaserScan & scan_in, LaserScan & scan_out) // Remove const from scan_in
  {
      // Copy metadata - important: keep original angle min/max/increment
      scan_out.header = scan_in.header; // Copy header (timestamp, frame_id)
      scan_out.angle_min = scan_in.angle_min;
      scan_out.angle_max = scan_in.angle_max;
      scan_out.angle_increment = scan_in.angle_increment;
      scan_out.time_increment = scan_in.time_increment;
      scan_out.scan_time = scan_in.scan_time;
      scan_out.range_min = scan_in.range_min;
      scan_out.range_max = scan_in.range_max;

      size_t num_ranges = scan_in.ranges.size();
      if (num_ranges == 0) {
          scan_out.ranges.clear();
          scan_out.intensities.clear();
          return; // Nothing to rotate
      }

      size_t half_size = num_ranges / 2;
      // Size of the first part (might be larger if num_ranges is odd)
      size_t first_half_size = num_ranges - half_size;

      // Resize output arrays
      scan_out.ranges.resize(num_ranges);
      bool has_intensities = scan_in.intensities.size() == num_ranges;
      if (has_intensities) {
          scan_out.intensities.resize(num_ranges);
      } else {
          scan_out.intensities.clear();
      }


      // --- Rotate ranges ---
      // Copy second half of input ranges [first_half_size, end) to beginning of output [0, half_size)
      std::copy(scan_in.ranges.begin() + first_half_size,
                scan_in.ranges.end(),
                scan_out.ranges.begin());
      // Copy first half of input ranges [0, first_half_size) to second half of output [half_size, end)
      std::copy(scan_in.ranges.begin(),
                scan_in.ranges.begin() + first_half_size,
                scan_out.ranges.begin() + half_size);

      // --- Rotate intensities (if they exist) ---
      if (has_intensities) {
          // Copy second half of input intensities to beginning of output
          std::copy(scan_in.intensities.begin() + first_half_size,
                    scan_in.intensities.end(),
                    scan_out.intensities.begin());
          // Copy first half of input intensities to second half of output
          std::copy(scan_in.intensities.begin(),
                    scan_in.intensities.begin() + first_half_size,
                    scan_out.intensities.begin() + half_size);
      }
  }

  /**
   * @brief Finds sequences of laser scan points that likely represent cans.
   * @param scan_in The (rotated) LaserScan message to analyze.
   * @param can_scan_out The LaserScan message containing only points identified as cans.
   * @param found_cans_out The PoseArray message to store the calculated positions of found cans.
   * @param potential_cans_out Output vector to store validated CanSequence objects.
   */
  void findCans(LaserScan & scan_in, LaserScan & can_scan_out, PoseArray & found_cans_out, std::vector<CanSequence>& potential_cans_out) // Remove const, add potential_cans_out
  {
      // Initialize output scan by copying input metadata
      can_scan_out = scan_in; // Copies header, angle info, etc.
      // Blank ranges and intensities with NaN
      float nan_value = std::numeric_limits<float>::quiet_NaN();
      std::fill(can_scan_out.ranges.begin(), can_scan_out.ranges.end(), nan_value);
      if (!can_scan_out.intensities.empty()) {
          std::fill(can_scan_out.intensities.begin(), can_scan_out.intensities.end(), nan_value);
      }

      // Initialize PoseArray header (frame_id and stamp set in scanCb)
      found_cans_out.header = scan_in.header;
      found_cans_out.poses.clear(); // Ensure it's empty
      potential_cans_out.clear(); // Clear output vector

      size_t num_ranges = scan_in.ranges.size();
      // Need at least 2 points for range diff check, maybe 3 for midpoint check logic.
      if (num_ranges < 3) return;

      float angle_increment = scan_in.angle_increment;
      if (angle_increment <= 0) {
           RCLCPP_WARN(this->get_logger(), "Invalid angle_increment: %f. Cannot find cans.", angle_increment);
           return;
      }

      std::vector<CanSequence> potential_cans;
      size_t current_sequence_start = 0;
      size_t valid_points_in_sequence = 0;
      float last_valid_range = -1.0; // Store the last finite range value
      bool in_sequence = false;

      // Iterate through the rotated scan data
      for (size_t i = 0; i < num_ranges; ++i) {
          float current_range = scan_in.ranges[i];
          // Check if the point is finite and within the sensor's valid range limits
          bool is_point_valid_for_sequence = std::isfinite(current_range) &&
                                             current_range >= scan_in.range_min &&
                                             current_range <= scan_in.range_max &&
                                             current_range < MAX_CAN_RANGE; // Add max can range check here

          if (is_point_valid_for_sequence) {
              if (!in_sequence) {
                  // Start of a new potential sequence
                  in_sequence = true;
                  current_sequence_start = i;
                  valid_points_in_sequence = 1;
                  last_valid_range = current_range;
              } else {
                  // Continue existing sequence - check range difference
                  if (std::abs(current_range - last_valid_range) <= MAX_RANGE_DIFF) {
                      // Point continues the sequence smoothly
                      valid_points_in_sequence++;
                      last_valid_range = current_range; // Update last valid range
                  } else {
                      // Range jump detected, end the previous sequence
                      validateAndAddSequence(scan_in, current_sequence_start, i - 1, potential_cans);
                      // Start a new sequence at the current point
                      current_sequence_start = i;
                      valid_points_in_sequence = 1;
                      last_valid_range = current_range;
                  }
              }
          } else { // Point is NaN, out of sensor range, or > MAX_CAN_RANGE
              if (in_sequence) {
                  // End the current sequence if we hit an invalid point
                  validateAndAddSequence(scan_in, current_sequence_start, i - 1, potential_cans);
                  in_sequence = false;
                  valid_points_in_sequence = 0;
                  last_valid_range = -1.0;
              }
              // Otherwise, just continue (not in a sequence)
          }
      }
      // Check if a sequence was ongoing at the end of the scan
      if (in_sequence) {
          validateAndAddSequence(scan_in, current_sequence_start, num_ranges - 1, potential_cans);
      }

      // Sort valid sequences by range (closest first)
      std::sort(potential_cans.begin(), potential_cans.end());

      // Populate output scan and pose array from validated and sorted sequences
      for (const auto& can : potential_cans) {
          // Populate output scan ranges/intensities for this can sequence
          for (size_t i = can.start_index; i <= can.end_index; ++i) {
               // Only copy valid points from the original scan that were part of the sequence
               if (std::isfinite(scan_in.ranges[i])) {
                  can_scan_out.ranges[i] = scan_in.ranges[i];
                  // Copy intensity if available
                  if (i < scan_in.intensities.size() && i < can_scan_out.intensities.size()) {
                      can_scan_out.intensities[i] = scan_in.intensities[i];
                  }
               }
          }

          // Calculate pose for the midpoint
          float mid_range = can.mid_range; // Use the validated midpoint range
          size_t mid_index = can.mid_index; // Use the validated midpoint index

          // Calculate angle theta relative to the lidar's forward direction (X-axis)
          // The input scan is rotated by PI.
          // theta = (midpoint_index_in_rotated_scan * angle_increment) + PI
          double theta = (static_cast<double>(mid_index) * angle_increment) + PI;
          // Normalize angle to [-pi, pi] for consistency, although cos/sin handle periodicity
          theta = atan2(sin(theta), cos(theta));

          geometry_msgs::msg::Pose pose;
          // Convert polar (mid_range, theta) to Cartesian (x, y)
          // x = r * cos(theta), y = r * sin(theta)
          pose.position.x = static_cast<double>(mid_range) * cos(theta);
          pose.position.y = static_cast<double>(mid_range) * sin(theta);
          pose.position.z = 0.0; // Assume 2D lidar
          // Set orientation to zero rotation (identity quaternion)
          pose.orientation.x = 0.0;
          pose.orientation.y = 0.0;
          pose.orientation.z = 0.0;
          pose.orientation.w = 1.0;

          found_cans_out.poses.push_back(pose);
      }
       // Optional: Log number of cans found
       // RCLCPP_DEBUG(this->get_logger(), "Found %zu cans in this scan.", found_cans_out.poses.size());
  }

  /**
   * @brief Validates a potential sequence of points based on can properties.
   * If valid, adds it to the list of potential cans.
   * @param scan The input laser scan data (rotated).
   * @param start_idx The starting index of the sequence in the scan data.
   * @param end_idx The ending index of the sequence in the scan data.
   * @param potential_cans The vector to add the validated sequence to.
   */
  void validateAndAddSequence(
      LaserScan& scan, // Remove const from scan
      size_t start_idx,
      size_t end_idx,
      std::vector<CanSequence>& potential_cans)
  {
      if (end_idx < start_idx) return; // Invalid indices

      // --- Find midpoint range and index, counting valid points ---
      size_t valid_points_count = 0;
      size_t first_valid_idx = end_idx + 1; // Initialize to invalid
      float first_valid_range = std::numeric_limits<float>::quiet_NaN();
      float last_valid_range = std::numeric_limits<float>::quiet_NaN();
      std::vector<std::pair<float, size_t>> valid_points_in_seq; // Store {range, index}

      for (size_t i = start_idx; i <= end_idx; ++i) {
          float r = scan.ranges[i];
          if (std::isfinite(r) && r < MAX_CAN_RANGE) { // Check MAX_CAN_RANGE here too
              valid_points_count++;
              valid_points_in_seq.push_back({r, i});
              if (first_valid_idx > end_idx) { // Found first valid point
                  first_valid_idx = i;
                  first_valid_range = r;
              }
              last_valid_range = r;
          }
      }

      // Need at least 2 valid points for further checks
      if (valid_points_count < 2) {
          // RCLCPP_DEBUG(this->get_logger(), "Sequence [%zu, %zu] failed: Not enough valid points (%zu)", start_idx, end_idx, valid_points_count);
          return;
      }

      // --- Determine Midpoint ---
      // Use the median valid point as the midpoint representative
      std::sort(valid_points_in_seq.begin(), valid_points_in_seq.end()); // Sort by range
      size_t median_idx_in_valid = valid_points_in_seq.size() / 2;
      float mid_range = valid_points_in_seq[median_idx_in_valid].first;
      size_t mid_index = valid_points_in_seq[median_idx_in_valid].second;


      // --- Rule: Range must be less than MAX_CAN_RANGE (already checked for points, check midpoint again) ---
      if (mid_range >= MAX_CAN_RANGE) {
           // RCLCPP_DEBUG(this->get_logger(), "Sequence [%zu, %zu] failed: Mid range %.3f >= MAX_CAN_RANGE %.2f", start_idx, end_idx, mid_range, MAX_CAN_RANGE);
          return;
      }

      // --- Rule: Number of valid points check ---
      double angle_increment = scan.angle_increment;
      // Calculate expected angle subtended by min/max can radius at mid_range
      double min_angle_subtended = 2.0 * atan(MIN_CAN_RADIUS_DETECT / mid_range);
      double max_angle_subtended = 2.0 * atan(MAX_CAN_RADIUS_DETECT / mid_range);
      // Calculate expected number of points based on angle and increment
      // Add 1 because N points span N-1 increments, but we count points.
      size_t min_points_expected = static_cast<size_t>(std::floor(min_angle_subtended / angle_increment)) +1;
      size_t max_points_expected = static_cast<size_t>(std::ceil(max_angle_subtended / angle_increment)) + 1;

      // Ensure min_points is at least 2, as required earlier
      min_points_expected = std::max(min_points_expected, (size_t)2);

      if (valid_points_count < min_points_expected || valid_points_count > max_points_expected) {
           // RCLCPP_DEBUG(this->get_logger(), "Sequence [%zu, %zu] failed: Point count %zu not in range [%zu, %zu] for range %.3f", start_idx, end_idx, valid_points_count, min_points_expected, max_points_expected, mid_range);
          return;
      }

      // --- Rule: Range to midpoint must be less than range to either end ---
      // Use the first and last *valid* points found earlier
      if (mid_range >= first_valid_range || mid_range >= last_valid_range) {
          // RCLCPP_DEBUG(this->get_logger(), "Sequence [%zu, %zu] failed: Mid range %.3f not less than ends (%.3f, %.3f)", start_idx, end_idx, mid_range, first_valid_range, last_valid_range);
          return;
      }

      // --- All rules passed ---
      // RCLCPP_DEBUG(this->get_logger(), "Sequence [%zu, %zu] PASSED validation. Mid range: %.3f, Points: %zu (expected [%zu, %zu])", start_idx, end_idx, mid_range, valid_points_count, min_points_expected, max_points_expected);
      potential_cans.push_back({start_idx, end_idx, mid_index, mid_range, valid_points_count});
  }

  // Member Variables
  rclcpp::Subscription<LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<LaserScan>::SharedPtr scan_publisher_;
  rclcpp::Publisher<LaserScan>::SharedPtr can_scan_publisher_;
  rclcpp::Publisher<PoseArray>::SharedPtr can_positions_publisher_;
  rclcpp::Publisher<Point>::SharedPtr closest_can_publisher_; // Publisher for closest can range/bearing
  rclcpp::Service<SetBool>::SharedPtr blank_fwd_sector_service_;
  bool blankFwdSector_; // Flag to control blanking (note the underscore)

  // TF2 related members
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<CanPoseTransformer> can_pose_transformer_; // Use unique_ptr
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
