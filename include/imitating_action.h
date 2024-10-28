#ifndef IMITAING_ACTION_H
#define IMITAING_ACTION_H

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "ai_msgs/msg/perception_targets.hpp"

struct Point{
    int x;
    int y;  

    //点与点之间的距离
    Point(int x_, int y_){
        this->x = x_;
        this->y = y_;
    }

    static int distance(const Point& p1, const Point& p2) {
        int x_d = p1.x - p2.x;
        int y_d = p1.y - p2.y;
        return sqrt(x_d * x_d + y_d * y_d);
    }
};

class ActionImitationNode : public rclcpp::Node{
public:
    ActionImitationNode(const std::string &node_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~ActionImitationNode() override;

private:
    rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr target_subscriber_;
    void subscription_callback(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg);
    int angle_calculator(const Point& point_1, const Point& point_2, const Point& point_3);
    std::shared_ptr<OrderInterpreter> order_interpreter_;

    std::string sub_topic_ = "hobot_mono2d_body_detection";
    std::string target_type_ = "red_ball";
    bool no_target = true;
};



#endif //IMITAING_ACTION_H