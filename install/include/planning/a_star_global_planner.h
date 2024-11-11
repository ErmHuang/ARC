#ifndef A_STAR_GLOBAL_PLANNER_H
#define A_STAR_GLOBAL_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <unordered_map>
#include <vector>
#include <string>
#include <queue>

namespace planning {

class AStarGlobalPlanner : public nav_core::BaseGlobalPlanner {
public:
    AStarGlobalPlanner();
    AStarGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
    struct Node {
        int x, y;
        double g_cost, h_cost;
        const Node* parent;

        Node() : x(0), y(0), g_cost(0.0), h_cost(0.0), parent(nullptr) {}  // 默认构造函数

        Node(int x_, int y_, double g, double h, const Node* p = nullptr);

        int getIndex(costmap_2d::Costmap2D* costmap) const {
            return y * costmap->getSizeInCellsX() + x;
        }
    };

    struct CompareCost {
        bool operator()(const Node& a, const Node& b) {
            return (a.g_cost + a.h_cost) > (b.g_cost + b.h_cost);
        }
    };

    void addNeighbors(const Node& current, int goal_x, int goal_y,
                      std::priority_queue<Node, std::vector<Node>, CompareCost>& open_list,
                      std::unordered_map<int, Node>& all_nodes, costmap_2d::Costmap2D* costmap);

    void reconstructPath(const Node& goal_node, const std::unordered_map<int, Node>& all_nodes,
                         std::vector<geometry_msgs::PoseStamped>& plan,
                         const geometry_msgs::PoseStamped& start,
                         const geometry_msgs::PoseStamped& goal);

    double heuristic(int x1, int y1, int x2, int y2);
    double distance(int x1, int y1, int x2, int y2);

    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    bool initialized_;
};

}  // namespace planning

#endif  // A_STAR_GLOBAL_PLANNER_H
