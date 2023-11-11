#ifndef LANE_TRACKING_PLANNER_H_
#define LANE_TRACKING_PLANNER_H_

#include <fstream>
#include <string>
#include <vector>
#include <limits>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <lane_tracking_planner/geometry_utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <angles/angles.h>
#include <geometry_msgs/Twist.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include <dynamic_reconfigure/client.h>
#include <mpc_local_planner/DynamicParamsConfig.h>

using namespace geometry_utils;

namespace lane_tracking_planner
{
    /**
     * @class LaneTrackingPlanner
     * @brief Provides a simple global planner that will compute a valid goal point for the local planner by walking back along the vector between the robot and the user-specified goal point until a valid cost is found.
     */
    class LaneTrackingPlanner : public nav_core::BaseGlobalPlanner
    {
    public:
        int tracking_lane_ = 0;
        int last_index_ = 0;
        std::vector<std::vector<geometry_msgs::PoseStamped>> lanes_;
        /**
         * @brief  Constructor for the LaneTrackingPlanner
         */
        LaneTrackingPlanner();
        /**
         * @brief  Constructor for the LaneTrackingPlanner
         * @param  name The name of this planner
         * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
         */
        LaneTrackingPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

        /**
         * @brief  Initialization function for the LaneTrackingPlanner
         * @param  name The name of this planner
         * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
         */
        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */

        bool makePlan(const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan);

        void initializePath(std::string path_file_name, int num_lane);

        void updateDesiredVelocity(int current_index);
        
        void updateVelocity();

        int find_current_index(const geometry_msgs::PoseStamped &current_pose, std::vector<geometry_msgs::PoseStamped> lane, int last_index);

        void checkLaneChange(const geometry_msgs::PoseStamped& current_pose);

        std::pair<bool, double> checkValidTrajectory(int tracking_lane, int start_index, int goal_index);

        int findValidTrajectory(int &tracking_lane, int start_index, int goal_index);

        void cmdVelCB(const geometry_msgs::Twist msg);

    private:
      costmap_2d::Costmap2DROS* costmap_ros_;
      double step_size_, min_dist_from_robot_;
      int num_lane_;
      int lookahead_index_;
      std::string path_file_name_{"file_name"};
      boost::shared_ptr<dynamic_reconfigure::Client<mpc_local_planner::DynamicParamsConfig>> reconfigure_client;
      mpc_local_planner::DynamicParamsConfig config;


        double vel_path_;
        double vel_lc_;
        int current_zone_index_;

        bool is_lane_changing_;

        costmap_2d::Costmap2D *costmap_;
        base_local_planner::WorldModel *world_model_; ///< @brief The world model that the controller will use

      
        
        // Publisher & Subscriber
        ros::Publisher plan_pub_;
        ros::Publisher cmd_vel_pub_;
        ros::Subscriber cmd_sub_;

        geometry_msgs::Twist current_cmd_;

        /**
         * @brief  Checks the legality of the robot footprint at a position and orientation using the world model
         * @param x_i The x position of the robot
         * @param y_i The y position of the robot
         * @param theta_i The orientation of the robot
         * @return
         */
        double footprintCost(double x_i, double y_i, double theta_i);

        bool initialized_;
    };
};
#endif
