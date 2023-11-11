#include <lane_tracking_planner/lane_tracking_planner.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>
#include <algorithm>
#include <lane_tracking_planner/geometry_utils.h>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(lane_tracking_planner::LaneTrackingPlanner, nav_core::BaseGlobalPlanner)

namespace lane_tracking_planner
{

    LaneTrackingPlanner::LaneTrackingPlanner()
        : costmap_ros_(NULL), initialized_(false), vel_path_(0.0), vel_lc_(5.0), current_zone_index_(-1), is_lane_changing_(false) {}

    LaneTrackingPlanner::LaneTrackingPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
        : costmap_ros_(NULL), initialized_(false), vel_path_(0.0), vel_lc_(5.0), current_zone_index_(-1), is_lane_changing_(false)
    {
        initialize(name, costmap_ros);
    }

    void LaneTrackingPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_)
        {
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();

            ros::NodeHandle private_nh("~/" + name);
            ROS_INFO("%s", name.c_str());

            // Initialize ROS Parameter
            private_nh.param<int>("num_lane", num_lane_, 0);
            private_nh.param<int>("lookahead_index", lookahead_index_, 100);
            private_nh.param<std::string>("path_file_name", path_file_name_, "default_file_name");

            world_model_ = new base_local_planner::CostmapModel(*costmap_);

            // Initialize Path
            initializePath(path_file_name_, num_lane_);

            // Initialize Publish & Subscriber

            plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);

            ros::NodeHandle nh;
            cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);

            initialized_ = true;
        }
        else
            ROS_WARN("This planner has already been initialized... doing nothing");
    }

    void LaneTrackingPlanner::initializePath(std::string path_file_name, int num_lane)
    {

        if (num_lane < 1)
        {
            ROS_WARN("The number of lanes is not set properly, it must be 1 at least");
        }

        std::string directory_path = ros::package::getPath("lane_tracking_planner") + "/paths";

        for (int i = 0; i < num_lane; i++)
        {
            std::string file_path = directory_path + "/" + path_file_name + "_" + std::to_string(i) + ".txt";
            std::ifstream in(file_path);
            std::vector<geometry_msgs::PoseStamped> lane;
            lane.reserve(1000);

            if (in.fail())
                throw std::ios_base::failure{"csv_read : cannot open file"};

            while (in.good())
                lane.push_back(csv_read_row(in, '\t'));

            lane.shrink_to_fit();

            lanes_.push_back(lane);
        }

        // ROS_INFO("Initialize %d lanes from pre-defined path file!", static_cast<int>(lanes_.size()));

        ROS_INFO("Intialize %d lanes from pre defined path file!", static_cast<int>(lanes_.size()));
    }

    void LaneTrackingPlanner::updateDesiredVelocity(int current_index)
    {
        int zone_idx = -1;
        if (current_index >= 165 && current_index < 291)
        {
            vel_path_ = 4.0;
            zone_idx = 1; // S-curve uphill
        }
        else if (current_index >= 291 && current_index < 534)
        {
            vel_path_ = 4.5;
            zone_idx = 2; // Straight uphill
        }
        else if (current_index >= 534 && current_index < 606)
        {
            vel_path_ = 4.5;
            zone_idx = 3; // U-curve
        }
        else if (current_index >= 606 && current_index < 846)
        {
            vel_path_ = 4.5;
            zone_idx = 4; // Straight downhill
        }
        else if (current_index >= 846 && current_index < 934)
        {
            vel_path_ = 4.5;
            zone_idx = 5; // S-curve downhill
        }
        else if (current_index >= 934 && current_index < 1180)
        {
            vel_path_ = 4.5;
            zone_idx = 6; // Straight 2
        }
        else if (current_index >= 1180 && current_index < 1218)
        {
            vel_path_ = 4.5;
            zone_idx = 7; // 90 curve 1
        }
        else if (current_index >= 1218 && current_index < 1300)
        {
            vel_path_ = 4.5;
            zone_idx = 8; // Straight narrow
        }
        else if (current_index >= 1300 && current_index < 1334)
        {
            vel_path_ = 4.5;
            zone_idx = 9; // 90 curve 2
        }
        else
        {
            vel_path_ = 5.0;
            zone_idx = 0; // Straight 1
        }
        if (current_zone_index_ != zone_idx)
        {
            current_zone_index_ = zone_idx;
            updateVelocity();
        }
    }

    void LaneTrackingPlanner::updateVelocity()
    {
        // set_param("desired_vel", std::min(vel_path_, vel_lc_)); // Have to be updated!!
        ROS_ERROR("vel_path: %.2f, vel_lc: %.2f, desired_vel: %.2f", vel_path_, vel_lc_, std::min(vel_path_, vel_lc_));
    }

    int LaneTrackingPlanner::find_current_index(const geometry_msgs::PoseStamped &current_pose, std::vector<geometry_msgs::PoseStamped> lane,
                                                int last_index = 0)
    {
        if (last_index >= lane.size())
            ROS_WARN("Get the invalid last_index!");

        int min_index;
        double min_dist = std::numeric_limits<double>::infinity();

        // If the vehicle is near by index 0, then search every index of the path
        if (last_index > lane.size() - lookahead_index_)
            last_index = 0;

        // Find the nearest point with current pose from last_index to end of the lane
        for (int i = last_index; i < lane.size(); i++)
        {
            double dist = euclidean_distance(current_pose, lane[i]);
            if (dist < min_dist)
            {
                min_dist = dist;
                min_index = i;
            }
        }
        return min_index;
    }

    void LaneTrackingPlanner::checkLaneChange(const geometry_msgs::PoseStamped& current_pose) {
        int nearest_index = find_current_index(current_pose, lanes_[tracking_lane_], last_index_);
        double distance = euclidean_distance(current_pose, lanes_[tracking_lane_][nearest_index]);

        if (is_lane_changing_ && distance < 0.5) {
            // lane changed
            is_lane_changing_ = false;
            vel_lc_ = vel_path_;
            updateVelocity();
        } else if (!is_lane_changing_ && distance >= 0.5) {
            // lane changing
            is_lane_changing_ = true;
        }
    }

    // we need to take the footprint of the robot into account when we calculate cost to obstacles
    double LaneTrackingPlanner::footprintCost(double x_i, double y_i, double theta_i)
    {
        if (!initialized_)
        {
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return -1.0;
        }

        std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
        // if we have no footprint... do nothing
        if (footprint.size() < 3)
            return -1.0;

        // check if the footprint is legal
        double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
        return footprint_cost;
    }

    bool LaneTrackingPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                                       const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
    {

        if (!initialized_)
        {
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return false;
        }

        ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

        plan.clear();
        costmap_ = costmap_ros_->getCostmap();
        // if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
        //   ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
        //       costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
        //   return false;
        // }

        // Find the nearest index and update last index
        int current_index = find_current_index(start, lanes_[tracking_lane_], last_index_);
        // ROS_INFO("current_index: %d", current_index);
        updateDesiredVelocity(current_index);
        checkLaneChange(start);
        last_index_ = current_index;

        // Compare goal and the point of lookahead index
        int goal_index = find_current_index(goal, lanes_[tracking_lane_], last_index_);
        if (goal_index < current_index)
            goal_index += lanes_[tracking_lane_].size();
        goal_index = goal_index < current_index + lookahead_index_ ? goal_index : current_index + lookahead_index_;

        // Change lane if the trajectory is not valid
        if (!checkValidTrajectory(tracking_lane_, current_index, goal_index).first)
        {
            // Check other lane and return false if it fails
            if (!findValidTrajectory(tracking_lane_, current_index, goal_index))
                return false;
            else
            {
                int current_index = find_current_index(start, lanes_[tracking_lane_], last_index_);
                last_index_ = current_index;
            }
        }

        // Assign plan vector
        plan = getRingTrajectory(lanes_[tracking_lane_], current_index, goal_index);
        plan = getInterpolatedTrajectory(plan, start);

        // Publish global plan for visualization
        nav_msgs::Path plan_msg;
        plan_msg.header.stamp = ros::Time::now();
        plan_msg.header.frame_id = costmap_ros_->getGlobalFrameID();
        plan_msg.poses = plan;
        plan_pub_.publish(plan_msg);

        bool done = true;
        return (done);
    }

    std::pair<bool, double> LaneTrackingPlanner::checkValidTrajectory(int tracking_lane, int start_index, int goal_index)
    {
        bool is_valid = true;
        double valid_distance = 0.0;

        for (int i = start_index; i < goal_index; i++)
        {
            int i_tmp = i % lanes_[tracking_lane].size(); // Wrap around if index exceeds lane size
            const auto &current_point = lanes_[tracking_lane][i_tmp];
            const auto &next_point = lanes_[tracking_lane][(i_tmp + 1) % lanes_[tracking_lane].size()];

            // Calculate the distance from the start_index to the current point
            if (i == start_index)
            {
                valid_distance = 0.0;
            }
            else
            {
                valid_distance += hypot(
                    lanes_[tracking_lane][i_tmp].pose.position.x - lanes_[tracking_lane][i_tmp - 1].pose.position.x,
                    lanes_[tracking_lane][i_tmp].pose.position.y - lanes_[tracking_lane][i_tmp - 1].pose.position.y);
            }

            double x_diff = next_point.pose.position.x - current_point.pose.position.x;
            double y_diff = next_point.pose.position.y - current_point.pose.position.y;
            double target_yaw = atan2(y_diff, x_diff);

            double footprint_cost = footprintCost(current_point.pose.position.x, current_point.pose.position.y, target_yaw);

            if (footprint_cost >= 200)
            {
                ROS_WARN("Invalid trajectory at distance: %.4f with footprint_cost: %.4f", valid_distance, footprint_cost);
                is_valid = false;
                break;
            }
        }

        return std::make_pair(is_valid, valid_distance);
    }

    int LaneTrackingPlanner::findValidTrajectory(int &tracking_lane, int start_index, int goal_index)
    {
        bool done = false;
        double max_valid_distance = checkValidTrajectory(tracking_lane, start_index, goal_index).second;
        int best_lane = tracking_lane; // Initialize with current lane as the default
        double stop_threshold = 10.0;

        // Check adjacent lanes
        for (int i = 1; i < lanes_.size(); i++)
        {
            // Check left side lanes
            if (tracking_lane - i >= 0)
            {
                auto result = checkValidTrajectory(tracking_lane - i, start_index, goal_index);
                bool path_valid = result.first;
                double distance_to_invalid_point = result.second;
                if (path_valid)
                {
                    tracking_lane = tracking_lane - i;
                    ROS_WARN("Change lane to %d", tracking_lane);
                    // Reduce the velocity during lane change
                    vel_lc_ = vel_path_ * 0.5;
                    updateVelocity();
                    done = true;
                    break;
                }
                else if (distance_to_invalid_point > max_valid_distance && i == 1)
                {
                    max_valid_distance = distance_to_invalid_point;
                    best_lane = tracking_lane - i;
                }
            }
            // Check right side lanes
            if (tracking_lane + i < lanes_.size())
            {
                auto result = checkValidTrajectory(tracking_lane + i, start_index, goal_index);
                bool path_valid = result.first;
                double distance_to_invalid_point = result.second;
                if (path_valid)
                {
                    tracking_lane = tracking_lane + i;
                    ROS_WARN("Change lane to %d", tracking_lane);
                    // Reduce the velocity during lane change
                    vel_lc_ = vel_path_ * 0.5;
                    updateVelocity();
                    done = true;
                    break;
                }
                else if (distance_to_invalid_point > max_valid_distance && i == 1)
                {
                    max_valid_distance = distance_to_invalid_point;
                    best_lane = tracking_lane + i;
                }
            }
        }

        if (!done && max_valid_distance > 0)
        {
            tracking_lane = best_lane; // Assign the best possible lane even if it's not valid
            ROS_WARN("Change lane to %d with max valid distance %f", best_lane, max_valid_distance);
            // Reduce the velocity during lane change
            vel_lc_ = vel_path_ * 0.5;
            updateVelocity();
            done = true;
        }

        // Stop the vehicle if no path is valid and the invalid distance is below a certain threshold
        if (max_valid_distance <= stop_threshold)
        {
            geometry_msgs::Twist stop_msg;
            stop_msg.linear.x = 0.0;
            stop_msg.angular.z = 0.0;
            cmd_vel_pub_.publish(stop_msg);
            vel_lc_ = 0.0;
            updateVelocity();

            ROS_ERROR("No valid path found and below stop threshold. Stopping the vehicle.");
            done = false;
        }

        // Return false if no lanes are valid and above the stop threshold
        return done;
    }
};
