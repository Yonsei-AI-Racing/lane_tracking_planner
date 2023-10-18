#ifndef GEOMETRY_UTILS_HPP_
#define GEOMETRY_UTILS_HPP_

#include <cmath>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <geometry_msgs/PoseStamped.h>

namespace geometry_utils
{

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
    point.header.frame_id = "map";

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

    return plan;
}

}  // namespace geometry_utils

#endif  // GEOMETRY_UTILS_HPP_