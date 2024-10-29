#include <thread>
#include <chrono>

#include "order_interpreter.hpp"
#include "imitating_action.h"

#define FILTER_NUM 2

ActionImitationNode::ActionImitationNode(const std::string& node_name, const rclcpp::NodeOptions& options) : rclcpp::Node(node_name, options) {
    
    this->declare_parameter<std::string>("sub_topic", sub_topic_);

    this->get_parameter<std::string>("sub_topic", sub_topic_);
    
    target_subscriber_ =  this->create_subscription<ai_msgs::msg::PerceptionTargets>(
      sub_topic_,
      1,
      std::bind(&ActionImitationNode::subscription_callback,
      this,
      std::placeholders::_1)); 
    // if (!msg_process_) {
    //     msg_process_ = std::make_shared<std::thread>(
    //         std::bind(&ActionImitationNode::MessageProcess, this));
    // }

    order_interpreter_ = std::make_shared<OrderInterpreter>();
    order_interpreter_->control_serial_servo("stand");
    order_interpreter_->control_PWM_servo(1, 1900, 200);
}
ActionImitationNode::~ActionImitationNode(){

}

double ActionImitationNode::angle_calculator(const Point& point_1, const Point& point_2, const Point& point_3){
    double l1 = Point::distance(point_2, point_3);
    double l2 = Point::distance(point_1, point_3);
    double l3 = Point::distance(point_1, point_2);

    double cos_2 = (l1 * l1 + l3 * l3 - l2 * l2) / (2 * l1 *l3);
    if (cos_2 < -1.0 || cos_2 > 1.0) {
        std::cerr << "The cosine value is out of range, unable to calculate angle!" << std::endl;
        return -1;
    }
    double radian = acos(cos_2);
    double degree = radian * (180.0 / M_PI);
    return degree;
}

void ActionImitationNode::angle_mean_filter(const double& angle, int& num, int& angle_sum, int& filter_result){
    angle_sum += angle;
    if(num > FILTER_NUM){
        filter_result = angle_sum / (FILTER_NUM + 1);
        angle_sum = 0;
        num = 0;
    }
    num++;
}

void ActionImitationNode::subscription_callback(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg){
    int max_size = 0;
    ai_msgs::msg::Target max_target;
    for(const auto &target : targets_msg->targets){
        if(target.rois[0].type == "body"){
            int size = target.rois[0].rect.height * target.rois[0].rect.width;
            if (size > max_size){
                max_target = target;
                max_size = size;
            }
        }
    }
    if(max_size == 0) return;
    Point p8 (max_target.points[0].point[8].x, max_target.points[0].point[8].y);
    Point p6 (max_target.points[0].point[6].x, max_target.points[0].point[6].y);
    Point p12 (max_target.points[0].point[12].x, max_target.points[0].point[12].y);
    Point p10 (max_target.points[0].point[10].x, max_target.points[0].point[10].y);

    if(p10.x > p6.x) return;
    double angle1 = angle_calculator(p8, p6, p12);
    angle_mean_filter(angle1, num1_, angle_sum1_, filter_result1_);
    int pluse1 = 850 - (750 / 180) * filter_result1_;
    order_interpreter_->control_serial_servo(7, pluse1, 0);


    double angle2 = angle_calculator(p6, p8, p10);
    if(angle2 == -1) angle2 = 180;

    angle_mean_filter(angle2, num2_, angle_sum2_, filter_result2_);
    double slope = double((p8.y - p6.y) )/ double((p8.x - p6.x));
    double val_y = slope * (p10.x - p6.x);
    // std::cout<<val_y<<" "<<(p10.y - p6.y)<<std::endl;
    int pluse2 = 0;
    if (val_y > (p10.y - p6.y)){
        pluse2 = 0 + (500 / 120) * (filter_result2_ - 60);
    } else {
        pluse2 = 900 - (400 / 120) * (filter_result2_ - 60);
    }
    order_interpreter_->control_serial_servo(6, pluse2, 0);
    // if (slope1 > slope2){
    //     pluse2 = 0 + (500 / 120) * (filter_result2_ - 60);
    // } else if(slope1 < slope2){
    //     pluse2 = 900 - (400 / 120) * (filter_result2_ - 60);
    // } else {
    //     pluse2 = 500;
    // }
    // std::cout<<pluse2<<std::endl;



    // Point p7 (max_target.points[0].point[7].x, max_target.points[0].point[7].y);
    // Point p5 (max_target.points[0].point[5].x, max_target.points[0].point[5].y);
    // Point p11 (max_target.points[0].point[11].x, max_target.points[0].point[11].y);
    // double angle = angle_calculator(p7, p5, p11);
    // int mean_angle = angle_mean_filter(angle);
    // int pluse = 150 + (750 / 180) * mean_angle;
    // order_interpreter_->control_serial_servo(15, pluse, 0);

    return;
}

// void ActionImitationNode::MessageProcess(){

// }


int main(int argc, char* argv[]){

    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ActionImitationNode>("ActionImitationNode"));

    rclcpp::shutdown();

    RCLCPP_WARN(rclcpp::get_logger("ActionImitationNode"), "Pkg exit");

    return 0;
}