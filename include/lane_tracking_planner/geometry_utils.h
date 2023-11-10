#ifndef GEOMETRY_UTILS_HPP_
#define GEOMETRY_UTILS_HPP_

#include <cmath>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <lane_tracking_planner/spline.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>

namespace geometry_utils
{

template <typename T>
void swap(T &x, T &y)
{
    T temp = std::move(x);
    x = std::move(y);
    y = std::move(temp);
}

std::string global_frame_id = "map";

inline double euclidean_distance(
  const geometry_msgs::Pose & pos1,
  const geometry_msgs::Pose & pos2)
{
  double dx = pos1.position.x - pos2.position.x;
  double dy = pos1.position.y - pos2.position.y;
  return std::hypot(dx, dy);
}

/**
 * @brief Get the L2 distance between 2 geometry_msgs::PoseStamped
 * @param pos1 First pose
 * @param pos1 Second pose
 * @return double L2 distance
 */
inline double euclidean_distance(
  const geometry_msgs::PoseStamped & pos1,
  const geometry_msgs::PoseStamped & pos2)
{
  return euclidean_distance(pos1.pose, pos2.pose);
}

auto csv_read_row(std::istream& in, char delimiter)
{
    std::stringstream ss;
    bool inquotes = false;
    bool packs = false;
    std::vector<double> row;
    geometry_msgs::PoseStamped point;
    point.header.frame_id = global_frame_id;

    while (in.good())
    {
        char c = in.get();

        switch (c)
        {
        case '"':
            if (!inquotes)      // begin quote char
                inquotes = true;
            else
            {
                if (in.peek() == '"')   // 2 consecutive quotes resolve to 1
                    ss << static_cast<char>(in.get());
                else                    // end quote char
                    inquotes = false;
            }
            break;

        case '{':
            if (!packs)
                packs = true;
            else
                throw std::ios_base::failure{ "csv_read_row : '{' duplicated" };
            break;

        case '}':
            if (!packs)
                throw std::ios_base::failure{ "csv_read_row : '{' must be followed by '}'" };
            else
                packs = false;
            break;

        case '\r': case '\n':
            if (!inquotes && !packs)
            {
                if (in.peek() == '\n')
                    in.get();

                row.push_back(std::stod(ss.str()));
                
                point.pose.position.x = row[0];
                point.pose.position.y = row[1];
                return point;
            }
            break;

        default:
            if (c == delimiter && !inquotes)
            {
                row.push_back(std::stod(ss.str()));
                ss.str("");
            }
            else
                ss << c;
            break;

        }
    }
}      

std::vector<geometry_msgs::PoseStamped> getRingTrajectory(std::vector<geometry_msgs::PoseStamped>& trajectory, int start_index, int goal_index)
{
    // Regularize vector index
    if(goal_index >= trajectory.size()) goal_index -= trajectory.size();
    
    std::vector<geometry_msgs::PoseStamped> plan;
    if(goal_index >= start_index){
        std::vector<geometry_msgs::PoseStamped> path(&trajectory[start_index], &trajectory[goal_index]);
        plan = path;
    }
    else{
        std::vector<geometry_msgs::PoseStamped> path(&trajectory[start_index], &trajectory[trajectory.size()-1]);
        std::vector<geometry_msgs::PoseStamped> path_2(&trajectory[0], &trajectory[goal_index]);
        path.insert(path.end(), path_2.begin(), path_2.end());
        plan = path;
    }

    tk::spline s;


    return plan;
}

std::vector<geometry_msgs::PoseStamped> getInterpolatedTrajectory(std::vector<geometry_msgs::PoseStamped> trajectory, geometry_msgs::PoseStamped start)
{
    int lookahead_index = 15;
    bool linear_interpolation = false;
    // If the length of trajectory is less than 12, then return the original trajectory
    if(trajectory.size() < lookahead_index+2) return trajectory;


    // Add reference points with start position(and next) and point after 10 index(and next index)

    double start_x = start.pose.position.x;
    double start_y = start.pose.position.y;
    double start_yaw = tf2::getYaw(start.pose.orientation);

    std::vector<double> ptsx, ptsy;
    ptsx.push_back(0.0);
    ptsy.push_back(0.0);

    ptsx.push_back(0.1);
    ptsy.push_back(0.0);

    for(int i=lookahead_index; i<lookahead_index+2; i++){
        double x_shift = trajectory[i].pose.position.x - start_x;
        double y_shift = trajectory[i].pose.position.y - start_y;
        double x_frenet = x_shift*cos(-start_yaw) - y_shift*sin(-start_yaw);
        double y_frenet = x_shift*sin(-start_yaw) + y_shift*cos(-start_yaw);
        ptsx.push_back(x_frenet);
        ptsy.push_back(y_frenet);
    }

    // If the x coordinate of 4th point is less then 3rd, then swap the position between two points.
    if(ptsx[2] > ptsx[3]) swap(ptsx[2], ptsx[3]);
    else if(ptsx[2] == ptsx[3]){
        linear_interpolation = true;
    }
    tk::spline s;
    // Make spline curve
    if(!linear_interpolation){
        s.set_points(ptsx, ptsy);
    }

    std::vector<geometry_msgs::PoseStamped> interpolated_trajectory;
    interpolated_trajectory.push_back(start);
    for(int i=1; i<lookahead_index; i++){
        geometry_msgs::PoseStamped point;
        point.header.frame_id = global_frame_id;
        double x_diff = ptsx[2] - ptsx[1];
        double x_shift = x_diff * i /10;
        double y_shift = linear_interpolation == true ?   (ptsy[2] - ptsy[1]) * (i /10) : s(x_shift);
        double x_world = (x_shift*cos(start_yaw) - y_shift*sin(start_yaw)) + start_x;
        double y_world = (x_shift*sin(start_yaw) + y_shift*cos(start_yaw)) + start_y;
        point.pose.position.x = x_world;
        point.pose.position.y = y_world;
        interpolated_trajectory.push_back(point);
    }

    // integrate interpolated path with tracking lane
    interpolated_trajectory.insert(interpolated_trajectory.end(), trajectory.begin()+lookahead_index, trajectory.end());

    return interpolated_trajectory;
}

}  // namespace geometry_utils

#endif  // GEOMETRY_UTILS_HPP_