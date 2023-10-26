#include <lane_tracking_planner/lane_tracking_planner.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(lane_tracking_planner::LaneTrackingPlanner, nav_core::BaseGlobalPlanner)



namespace lane_tracking_planner {

  LaneTrackingPlanner::LaneTrackingPlanner()
  : costmap_ros_(NULL), initialized_(false){}

  LaneTrackingPlanner::LaneTrackingPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), initialized_(false){
    initialize(name, costmap_ros);
  }
  
  void LaneTrackingPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();

      ros::NodeHandle private_nh("~/" + name);
      ROS_INFO("%s",name.c_str());

      // Initialize ROS Parameter
      private_nh.param<int>("num_lane", num_lane_, 0);
      private_nh.param<int>("lookahead_index", lookahead_index_, 100);
      private_nh.param<std::string>("path_file_name", path_file_name_, "default_file_name");

      world_model_ = new base_local_planner::CostmapModel(*costmap_); 

      // Initialize Path
      initializePath(path_file_name_, num_lane_);

      // Initialize Publish & Subscriber 

      plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);

      initialized_ = true;

    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  void LaneTrackingPlanner::initializePath(std::string path_file_name, int num_lane){

    if(num_lane < 1){
      ROS_WARN("The number of lanes is not set properly, it must be 1 at least");
    }

    std::string directory_path = ros::package::getPath("lane_tracking_planner") + "/paths";
    
    for(int i = 0; i < num_lane; i++){
      std::string file_path = directory_path + "/" + path_file_name + "_" + std::to_string(i) + ".txt";
      std::ifstream in(file_path);
      std::vector<geometry_msgs::PoseStamped> lane;
      lane.reserve(1000);
 
      if (in.fail())
          throw std::ios_base::failure{ "csv_read : cannot open file" };

      while (in.good())
          lane.push_back(csv_read_row(in, '\t'));

      lane.shrink_to_fit();

      lanes_.push_back(lane);
    }

    ROS_INFO("Intialize %d lanes from pre defined path file!", lanes_.size());
  }

  int LaneTrackingPlanner::find_current_index(const geometry_msgs::PoseStamped& current_pose , std::vector<geometry_msgs::PoseStamped> lane, 
      int last_index = 0)
  {
    if(last_index >= lane.size()) 
      ROS_WARN("Get the invalid last_index!");

    double min_index;
    double min_dist = std::numeric_limits<double>::infinity(); 

    // If the vehicle is near by index 0, then search every index of the path
    if(last_index > lane.size() - lookahead_index_) last_index = 0;

    // Find the nearest point with current pose from last_index to end of the lane 
    for(int i = last_index; i < lane.size(); i++){
      double dist = euclidean_distance(current_pose, lane[i]);
      if(dist < min_dist){
        min_dist = dist; 
        min_index = i;
      }
    }
    return min_index;
  }

  //we need to take the footprint of the robot into account when we calculate cost to obstacles
  double LaneTrackingPlanner::footprintCost(double x_i, double y_i, double theta_i){
    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return -1.0;
    }

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
    //if we have no footprint... do nothing
    if(footprint.size() < 3)
      return -1.0;

    //check if the footprint is legal
    double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
    return footprint_cost;
  }


  bool LaneTrackingPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

    if(!initialized_){
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

    // Find the nearest index and update last inex
    int current_index = find_current_index(start, lanes_[tracking_lane_], last_index_);
    last_index_ = current_index;

    // Compare goal and the point of lookahead index
    int goal_index = find_current_index(goal, lanes_[tracking_lane_], last_index_);
    if(goal_index < current_index) goal_index += lanes_[tracking_lane_].size();
    goal_index = goal_index < current_index + lookahead_index_ ? goal_index : current_index + lookahead_index_;

    

    // Change lane if the trajectory is not valid
    if(!checkValidTrajectory(tracking_lane_, current_index, goal_index)){
      // Check other lane and return false if it fails
      if(!findValidTrajectory(tracking_lane_, current_index, goal_index)) return false;
      else{
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

  bool LaneTrackingPlanner::checkValidTrajectory(int tracking_lane, int start_index, int goal_index){

    bool is_valid = true;
    
    double start_x = lanes_[tracking_lane][start_index].pose.position.x;
    double start_y = lanes_[tracking_lane][start_index].pose.position.y;
    for(int i = start_index; i < goal_index; i++)
    {
      int i_tmp = i;
      if(i_tmp >= lanes_[tracking_lane].size()) i_tmp -= lanes_[tracking_lane].size();
      double target_x = lanes_[tracking_lane][i_tmp].pose.position.x;
      double target_y = lanes_[tracking_lane][i_tmp].pose.position.y;
      double x_diff = lanes_[tracking_lane][i_tmp+1].pose.position.x - target_x;
      double y_diff = lanes_[tracking_lane][i_tmp+1].pose.position.y - target_y;
      double target_yaw = angles::normalize_angle(atan2(y_diff, x_diff));
      double footprint_cost = footprintCost(target_x, target_y, target_yaw);
      if(footprint_cost >= 200)
      {
        ROS_WARN("footprint_cost : %.4f", footprint_cost);
        is_valid = false;
        break;
      }
    }
    
    return is_valid;
  }
  int LaneTrackingPlanner::findValidTrajectory(int& tracking_lane, int start_index, int goal_index){
    bool done = false;

    // Check adjacent lanes
    for(int i = 0;i < lanes_.size(); i++){
      if(tracking_lane - i >= 0){
        if(checkValidTrajectory(tracking_lane - i, start_index, goal_index)){
          tracking_lane = tracking_lane - i;
          done = true;
          break;
        }
      }
      if(tracking_lane + i < lanes_.size()){
        if(checkValidTrajectory(tracking_lane + i, start_index, goal_index)){
          tracking_lane = tracking_lane + i;
          done = true;
          break;
        }
      } 
    }
    return done;
  }
};
