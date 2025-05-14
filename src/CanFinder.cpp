#include <functional>
#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <algorithm> // For std::sort, std::copy, std::fill
#include <chrono> // For transform timeout

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" // For transform
#include "example_interfaces/srv/set_bool.hpp"
#include "example_interfaces/msg/bool.hpp" // For Bool message
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // For tf2::doTransform

// Define PI if not already defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

/**
 * @brief Node to detect can-like objects from laser scan data.
 *
 * Subscribes to raw laser scans, filters them, identifies potential cans,
 * publishes the filtered scan, the scan points belonging to cans,
 * the positions of detected cans in the map frame, and the range/bearing
 * of the closest can. Provides a service to blank the forward sector of the scan.
 */
class CanFinder : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Can Finder object.
     * Initializes publishers, subscribers, service, and TF2 listener.
     */
    CanFinder()
        : Node("can_finder"), blankFwdSector_(false) // Initialize blankFwdSector_ to false
    {
        // Initialize TF2 buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscriber to unfiltered laser scan data
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan/unfiltered", 10, std::bind(&CanFinder::scanCb, this, _1));

        // Publisher for potentially modified scan data (e.g., blanked)
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

        // Publisher for scan data containing only points on detected cans
        can_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/can_scan", 10);

        // Publisher for the positions of detected cans (in map frame)
        can_positions_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/can_positions", 10);

        // Publisher for the range and bearing of the closest can (in laser frame)
        closest_can_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/closest_range_bearing", 10);

        // Publisher for can detection status
        can_detected_pub_ = this->create_publisher<example_interfaces::msg::Bool>("/can_detected", 10);

        // Service server to control forward sector blanking
        blank_fwd_sector_srv_ = this->create_service<example_interfaces::srv::SetBool>(
            "/blank_fwd_sector", std::bind(&CanFinder::blankFwdSectorCb, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "CanFinder node initialized.");
    }

private:
    /**
     * @brief Callback function for incoming laser scan messages.
     * Processes the scan to find cans and publishes results.
     * @param msg Shared pointer to the incoming LaserScan message.
     */
    void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Create a mutable copy for potential modification
        sensor_msgs::msg::LaserScan current_scan = *msg;

        // Blank forward sector if requested (e.g., when carrying a can)
        if (blankFwdSector_) {
            blankCapturedCan(current_scan);
        }
        // Publish the (potentially modified) scan
        scan_pub_->publish(current_scan);

        // Rotate the scan data by 180 degrees for easier processing of front-facing objects
        sensor_msgs::msg::LaserScan rotatedScan;
        rotateScan(current_scan, rotatedScan);

        // Find cans in the rotated scan data
        sensor_msgs::msg::LaserScan canScanRot; // Scan containing only can points (still rotated)
        geometry_msgs::msg::PoseArray foundCans; // Poses of found cans (in laser frame, relative to rotated scan origin)
        findCans(rotatedScan, canScanRot, foundCans);

        // Publish whether any cans were detected in the laser frame
        example_interfaces::msg::Bool can_detected_msg;
        can_detected_msg.data = !foundCans.poses.empty(); // Check if findCans found anything
        can_detected_pub_->publish(can_detected_msg);

        // Rotate the can-only scan back to the original orientation
        sensor_msgs::msg::LaserScan canScan;
        rotateScan(canScanRot, canScan);

        // Publish the can-only scan data
        can_scan_pub_->publish(canScan);

        // Transform the found can poses from the laser frame to the map frame
        transformCanPoses(foundCans); // Modifies foundCans in place

        // Publish the transformed can positions or an empty array if none were found
        // If no cans were found, the poses array will be empty
        can_positions_pub_->publish(foundCans);
    }

    /**
     * @brief Service callback to enable or disable forward sector blanking.
     * @param request The service request containing the boolean flag.
     * @param response The service response indicating success.
     */
    void blankFwdSectorCb(
        const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
        std::shared_ptr<example_interfaces::srv::SetBool::Response> response)
    {
        blankFwdSector_ = request->data;
        response->success = true;
        response->message = blankFwdSector_ ? "Forward sector blanking enabled" : "Forward sector blanking disabled";
        RCLCPP_INFO(this->get_logger(), response->message.c_str());
    }

    /**
     * @brief Blanks (sets to NaN) specific ranges in the laser scan, typically
     *        to ignore a carried object directly in front of the sensor.
     * @param scan The LaserScan message to modify in place.
     */
    void blankCapturedCan(sensor_msgs::msg::LaserScan& scan)
    {
        size_t num_ranges = scan.ranges.size();
        if (num_ranges == 0) return; // No data to blank

        float nan_val = std::numeric_limits<float>::quiet_NaN();

        // Blank ranges at the beginning (index 0 to 25)
        for (size_t i = 0; i <= 25 && i < num_ranges; ++i) {
            scan.ranges[i] = nan_val;
        }

        // Blank ranges at the end (index 430 to end)
        if (num_ranges > 430) {
            for (size_t i = 430; i < num_ranges; ++i) {
                scan.ranges[i] = nan_val;
            }
        }
        // Also blank intensities if they exist
        if (!scan.intensities.empty()) {
             for (size_t i = 0; i <= 25 && i < num_ranges; ++i) {
                scan.intensities[i] = 0.0f; // Or NaN if appropriate, but 0 is common
            }
            if (num_ranges > 430) {
                for (size_t i = 430; i < num_ranges; ++i) {
                    scan.intensities[i] = 0.0f;
                }
            }
        }
    }

    /**
     * @brief Rotates the laser scan data by 180 degrees.
     * This helps in processing objects directly in front, which might span
     * the beginning and end of the original scan array.
     * @param input_scan The original LaserScan message.
     * @param output_scan The LaserScan message to store the rotated data.
     */
    void rotateScan(const sensor_msgs::msg::LaserScan& input_scan,
                    sensor_msgs::msg::LaserScan& output_scan)
    {
        // Copy metadata
        output_scan.header = input_scan.header;
        output_scan.angle_min = input_scan.angle_min;
        output_scan.angle_max = input_scan.angle_max;
        output_scan.angle_increment = input_scan.angle_increment;
        output_scan.time_increment = input_scan.time_increment;
        output_scan.scan_time = input_scan.scan_time;
        output_scan.range_min = input_scan.range_min;
        output_scan.range_max = input_scan.range_max;

        size_t num_ranges = input_scan.ranges.size();
        if (num_ranges == 0) return; // Nothing to rotate

        size_t half_size = num_ranges / 2;
        size_t first_half_count = half_size;
        size_t second_half_count = num_ranges - half_size;

        // Resize output vectors
        output_scan.ranges.resize(num_ranges);
        if (!input_scan.intensities.empty()) {
            output_scan.intensities.resize(num_ranges);
        } else {
            output_scan.intensities.clear();
        }


        // Copy second half of input ranges to first half of output ranges
        std::copy(input_scan.ranges.begin() + first_half_count, input_scan.ranges.end(),
                  output_scan.ranges.begin());
        // Copy first half of input ranges to second half of output ranges
        std::copy(input_scan.ranges.begin(), input_scan.ranges.begin() + first_half_count,
                  output_scan.ranges.begin() + second_half_count);

        // Repeat for intensities if they exist
        if (!input_scan.intensities.empty()) {
            std::copy(input_scan.intensities.begin() + first_half_count, input_scan.intensities.end(),
                      output_scan.intensities.begin());
            std::copy(input_scan.intensities.begin(), input_scan.intensities.begin() + first_half_count,
                      output_scan.intensities.begin() + second_half_count);
        }
    }

    /**
     * @brief Finds sequences of laser scan points that likely represent cans.
     * Applies geometric filters based on expected can size and range continuity.
     * @param rotated_scan The input LaserScan data (rotated 180 degrees).
     * @param can_scan_rot Output LaserScan containing only points belonging to detected cans (still rotated).
     * @param found_cans Output PoseArray containing the calculated poses of detected cans in the laser frame.
     */
    void findCans(const sensor_msgs::msg::LaserScan& rotated_scan,
                  sensor_msgs::msg::LaserScan& can_scan_rot,
                  geometry_msgs::msg::PoseArray& found_cans)
    {
        // Initialize output scan by copying input metadata and setting ranges to NaN
        can_scan_rot = rotated_scan;
        float nan_val = std::numeric_limits<float>::quiet_NaN();
        std::fill(can_scan_rot.ranges.begin(), can_scan_rot.ranges.end(), nan_val);
        if (!can_scan_rot.intensities.empty()) {
             std::fill(can_scan_rot.intensities.begin(), can_scan_rot.intensities.end(), 0.0f); // Or NaN
        }


        const auto& ranges = rotated_scan.ranges;
        size_t num_ranges = ranges.size();
        if (num_ranges == 0 || rotated_scan.angle_increment == 0.0) {
             RCLCPP_WARN_ONCE(this->get_logger(), "Invalid scan data received (0 ranges or 0 angle_increment).");
             return;
        }


        // Structure to hold information about potential can sequences
        struct CanSequence {
            size_t start_idx; // Index of the first valid point in the sequence
            size_t end_idx;   // Index of the last valid point in the sequence
            size_t mid_idx;   // Index of the middle valid point
            float mid_range;  // Range of the middle valid point
            int valid_points_count; // Count of non-NaN points within range limits
            float start_range;// Range of the first valid point
            float end_range;  // Range of the last valid point
            std::vector<size_t> valid_indices; // Store indices of valid points within bounds
        };
        std::vector<CanSequence> valid_sequences;

        size_t current_sequence_start_idx = 0; // Tentative start index
        int current_valid_points_in_seq = 0; // Count of valid points in current sequence
        float last_valid_range_in_seq = -1.0f; // Range of the last valid point added

        for (size_t i = 0; i < num_ranges; ++i) {
            float current_range = ranges[i];
            bool is_point_valid_for_seq = false; // Is this point potentially part of a can sequence?
            bool break_sequence = true; // Assume sequence breaks unless conditions met

            // Basic validity check (not NaN/inf, within sensor limits)
            bool is_point_valid_basic = !std::isnan(current_range) && !std::isinf(current_range) &&
                                        current_range >= rotated_scan.range_min && current_range <= rotated_scan.range_max;

            if (is_point_valid_basic) {
                // Check application-specific limits (max range for cans)
                if (current_range <= MAX_CAN_RANGE) {
                    // Check continuity with the *last valid point* in the sequence
                    if (current_valid_points_in_seq == 0 || std::abs(current_range - last_valid_range_in_seq) <= MAX_RANGE_DIFF) {
                        is_point_valid_for_seq = true;
                        break_sequence = false; // Point continues the sequence
                    }
                    // else: Point is valid but range jump is too large, breaks sequence
                }
                // else: Point is valid but too far for a can, breaks sequence
            }
            // else: Point is NaN/inf/outside sensor limits.
            // Allow NaNs within a sequence (don't break immediately)
            else if (std::isnan(current_range) && current_valid_points_in_seq > 0) {
                 break_sequence = false; // Don't break on NaN if sequence is active
            }


            // --- Sequence Start/Continuation ---
            if (is_point_valid_for_seq) {
                if (current_valid_points_in_seq == 0) { // Start of a new potential sequence
                    current_sequence_start_idx = i;
                }
                current_valid_points_in_seq++;
                last_valid_range_in_seq = current_range; // Update last valid range *within the sequence*
            }

            // --- Sequence End/Evaluation ---
            // Evaluate if:
            // 1. Sequence break detected AND there was a sequence ongoing (current_valid_points_in_seq > 0)
            // 2. We reached the end of the scan AND there was a sequence ongoing
            if ((break_sequence && current_valid_points_in_seq > 0) || (i == num_ranges - 1 && current_valid_points_in_seq > 0)) {
                size_t sequence_end_idx_tentative = (break_sequence) ? i - 1 : i; // Tentative end index

                // --- Refine sequence bounds and find valid points ---
                size_t actual_start_idx = num_ranges; // Initialize to invalid
                size_t actual_end_idx = 0;           // Initialize to invalid
                size_t mid_idx = 0;
                float mid_range = 0.0f;
                int non_nan_count_in_bounds = 0;
                std::vector<size_t> valid_indices_in_bounds; // Store indices of valid points

                for(size_t k = current_sequence_start_idx; k <= sequence_end_idx_tentative; ++k) {
                    // Check if point k is valid *and* within can range limit
                    if (!std::isnan(ranges[k]) && !std::isinf(ranges[k]) && ranges[k] <= MAX_CAN_RANGE) {
                         if (actual_start_idx == num_ranges) actual_start_idx = k; // Found first valid
                         actual_end_idx = k; // Update last valid
                         non_nan_count_in_bounds++;
                         valid_indices_in_bounds.push_back(k);
                    }
                }

                // --- Validate the refined sequence ---
                if (non_nan_count_in_bounds > 0) {
                    // Calculate midpoint index and range (use median index of valid points)
                    mid_idx = valid_indices_in_bounds[non_nan_count_in_bounds / 2];
                    mid_range = ranges[mid_idx];

                    // Calculate expected size based on midpoint range
                    if (mid_range > 0) { // Ensure midpoint range is valid
                        double angle_increment = rotated_scan.angle_increment;
                        // Use actual can radius (0.033) for calculations
                        double min_angle_subtended = 2.0 * atan(CAN_DIAMETER_MIN / 2.0 / mid_range);
                        double max_angle_subtended = 2.0 * atan(CAN_DIAMETER_MAX / 2.0 / mid_range);
                        // Add tolerance (+1) to max points expected? Or adjust diameter slightly?
                        // Let's stick to the formula for now.
                        double min_points_expected = min_angle_subtended / angle_increment;
                        double max_points_expected = max_angle_subtended / angle_increment;

                        float start_range = ranges[actual_start_idx];
                        float end_range = ranges[actual_end_idx];

                        // Apply validation rules
                        if (non_nan_count_in_bounds >= min_points_expected &&
                            non_nan_count_in_bounds <= max_points_expected &&
                            mid_range < start_range && // Midpoint closer than start
                            mid_range < end_range)     // Midpoint closer than end
                        {
                            // Valid can sequence found
                            valid_sequences.push_back({actual_start_idx, actual_end_idx, mid_idx, mid_range, non_nan_count_in_bounds, start_range, end_range, valid_indices_in_bounds});
                        }
                    }
                }
                // Reset for next potential sequence regardless of validation outcome
                current_valid_points_in_seq = 0;
                last_valid_range_in_seq = -1.0f;
            }

             // --- Handle the case where the current point *starts* a new sequence after a break ---
             // This needs to happen *after* evaluating the previous sequence.
             if (break_sequence && is_point_valid_for_seq) {
                 current_sequence_start_idx = i;
                 current_valid_points_in_seq = 1;
                 last_valid_range_in_seq = current_range;
             }
        } // End of scan loop

        // --- Process and Publish Valid Sequences ---
        if (!valid_sequences.empty()) {
            // Sort sequences by midpoint range (closest first)
            std::sort(valid_sequences.begin(), valid_sequences.end(),
                      [](const CanSequence& a, const CanSequence& b) {
                          return a.mid_range < b.mid_range;
                      });

            // Populate found_cans message header
            found_cans.header = rotated_scan.header; // Use laser frame initially
            found_cans.poses.reserve(valid_sequences.size());

            // Populate can_scan_rot and found_cans
            for (const auto& seq : valid_sequences) {
                // Mark points in can_scan_rot using the stored valid indices
                for (size_t valid_idx : seq.valid_indices) {
                    can_scan_rot.ranges[valid_idx] = ranges[valid_idx];
                    if (!can_scan_rot.intensities.empty() && valid_idx < rotated_scan.intensities.size()) {
                        can_scan_rot.intensities[valid_idx] = rotated_scan.intensities[valid_idx];
                    }
                }


                // Calculate pose for the can midpoint
                // Angle calculation correction: angle_min is the angle of the *first* measurement (index 0).
                // The angle needs to be relative to the lidar's coordinate system (X forward).
                // The rotation means index 0 corresponds to angle_min + pi.
                // However, the findCans operates on the *rotated* scan. The angle calculation
                // should be relative to the *rotated* scan's frame.
                // Let's calculate angle relative to the rotated scan's start (which is physically 180 deg).
                // Calculate angle relative to the start of the rotated scan data
                double angle_in_rotated_frame = rotated_scan.angle_min + seq.mid_idx * rotated_scan.angle_increment;
                // Convert angle to the original laser frame (X forward) by adding PI
                double angle_in_original_frame = angle_in_rotated_frame + M_PI;
                // Normalize angle to [-PI, PI] range (optional, but good practice)
                angle_in_original_frame = atan2(sin(angle_in_original_frame), cos(angle_in_original_frame));


                // Calculate X and Y in the *original* laser frame (X forward, Y left)
                geometry_msgs::msg::Pose pose;
                pose.position.x = seq.mid_range * cos(angle_in_original_frame);
                pose.position.y = seq.mid_range * sin(angle_in_original_frame);
                pose.position.z = 0.0;
                // Orientation: identity quaternion (no rotation relative to laser frame)
                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = 0.0;
                pose.orientation.w = 1.0;
                found_cans.poses.push_back(pose);
            }

            // Publish range and bearing for the closest can (using the angle relative to the original frame)
            const auto& closest_can = valid_sequences[0];
            // Calculate angle relative to the start of the rotated scan data
            double closest_angle_in_rotated_frame = rotated_scan.angle_min + closest_can.mid_idx * rotated_scan.angle_increment;
            // Convert angle to the original laser frame (X forward) by adding PI
            double closest_angle_in_original_frame = closest_angle_in_rotated_frame + M_PI;
            // Normalize angle to [-PI, PI] range (optional, but good practice)
            closest_angle_in_original_frame = atan2(sin(closest_angle_in_original_frame), cos(closest_angle_in_original_frame));


            geometry_msgs::msg::Point range_bearing;
            range_bearing.x = closest_can.mid_range;            // Range
            range_bearing.y = closest_angle_in_original_frame;  // Bearing (angle in original frame)
            range_bearing.z = 0.0;                              // Not used
            closest_can_pub_->publish(range_bearing);
        }
    }


    /**
     * @brief Transforms the poses in a PoseArray from their original frame (laser) to the odom frame.
     * Modifies the input PoseArray in place.
     * @param found_cans The PoseArray message containing poses in the laser frame. Header must be set correctly.
     */
    void transformCanPoses(geometry_msgs::msg::PoseArray& found_cans)
    {
        std::string target_frame = "odom";
        found_cans.header.frame_id = target_frame; // Set the target frame for the output

        if (found_cans.poses.empty()) {
            return; // Nothing to transform
        }

        std::string source_frame = found_cans.header.frame_id; // Should be laser frame_id

        if (source_frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Source frame_id is empty in PoseArray header. Cannot transform.");
            found_cans.poses.clear(); // Clear poses as they cannot be transformed
            return;
        }

        // Store original poses temporarily
        auto original_poses = found_cans.poses;
        found_cans.poses.clear(); // Clear to repopulate with transformed poses

        geometry_msgs::msg::PoseStamped stamped_in, stamped_out;
        stamped_in.header = found_cans.header; // Use original header with timestamp and frame_id

        for (const auto& original_pose : original_poses) {
            stamped_in.pose = original_pose; // Assign the current pose

            try {
                // Use the timestamp from the original scan header for the transform lookup
                // Add a timeout to wait for the transform to become available.
                stamped_out = tf_buffer_->transform(stamped_in, target_frame, 200ms); // Wait up to 200ms
                found_cans.poses.push_back(stamped_out.pose); // Add transformed pose

            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Could not transform %s to %s: %s. Skipping pose.",
                             source_frame.c_str(), target_frame.c_str(), ex.what());
                // Optionally, decide whether to skip this pose or stop processing
            }
        }

        // Update the header frame_id only if transforms were successful (or partially successful)
        // Even if some transforms failed, the ones that succeeded are in the map frame.
        found_cans.header.frame_id = target_frame; // Set frame_id to map regardless of partial success

        if (found_cans.poses.empty() && !original_poses.empty()) {
             RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "All can pose transformations failed. Output PoseArray is empty.");
        }
    }

    // ROS2 Components
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr can_scan_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr can_positions_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr closest_can_pub_;
    rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr can_detected_pub_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr blank_fwd_sector_srv_;

    // TF2 Listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // State Variables
    bool blankFwdSector_; // Flag to control forward sector blanking

    // Constants for can detection logic
    // Use radii derived from diameters in the request (0.066 -> 0.033)
    // Request asks for 0.015 and 0.04 radii, corresponding to 0.030 and 0.080 diameters
    const double CAN_DIAMETER_MIN = 0.030; // Minimum expected diameter
    const double CAN_DIAMETER_MAX = 0.080; // Maximum expected diameter
    const double MAX_CAN_RANGE = 1.0;      // Maximum range at which to look for cans
    const double MAX_RANGE_DIFF = 0.02;    // Maximum allowed range difference between consecutive points in a can sequence
};

/**
 * @brief Main function to initialize and run the CanFinder node.
 */
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanFinder>());
    rclcpp::shutdown();
    return 0;
}
