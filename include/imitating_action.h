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
    double angle_calculator(const Point& point_1, const Point& point_2, const Point& point_3);

    // void angle_mean_filter(const double& angle, int& num, int& angle_sum, int& filter_result);
    void angle_mean_filter(const double& angle, int& num, std::vector<int>& angles, int& filter_result);

    bool collision_detection(const double& degree1, const double& degree2);

    bool collision_detection_pluse_r(const int& p6, const int& p7);
    bool collision_detection_pluse_l(const int& p14, const int& p15);

    void MessageProcess(void);
    std::shared_ptr<OrderInterpreter> order_interpreter_;

    std::string sub_topic_ = "/hobot_hand_gesture_detection";
    ai_msgs::msg::PerceptionTargets targets_msg_;
    std::mutex target_mutex_;
    std::mutex start_mutex_;
    bool process_stop_ = false;
    std::shared_ptr<std::thread> msg_process_;
    int num1_ = 0, num2_ = 0, num3_ = 0, num4_ = 0;
    int angle_sum1_ = 0, angle_sum2_ = 0, angle_sum3_ = 0, angle_sum4_ = 0;
    int filter_result1_ = 0, filter_result2_ = 180, filter_result3_ = 0, filter_result4_ = 180;
    int pluse_ = 2000;
    bool start_control_ = false;
    bool gesture_control_ = false;
    bool imitating_control_ = false;
    bool update_data_ = false;
    int end_num_ = 0;
    int start_num_ = 0;
    int right_control_num_ = 0;
    int left_control_num_ = 0;
    int offset_ = 70;
    float ratio_ = 0.15;

    int head_id_ = 0;
    int hand_id_ = 0;
    std::vector<int> angles1{6};
    std::vector<int> angles2{6};
    std::vector<int> angles3{6};
    std::vector<int> angles4{6};
};



#endif //IMITAING_ACTION_H