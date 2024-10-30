#include <thread>
#include <chrono>

#include "order_interpreter.hpp"
#include "imitating_action.h"

#define POINT_FILTER_NUM 1


ActionImitationNode::ActionImitationNode(const std::string& node_name, const rclcpp::NodeOptions& options) : rclcpp::Node(node_name, options) {
    
    this->declare_parameter<std::string>("sub_topic", sub_topic_);
    this->declare_parameter<int>("a", a);
    this->declare_parameter<int>("pluse", pluse_);
    this->declare_parameter<float>("b", b);

    this->get_parameter<int>("a", a);
    this->get_parameter<int>("pluse", pluse_);
    this->get_parameter<float>("b", b);
    this->get_parameter<std::string>("sub_topic", sub_topic_);
    order_interpreter_ = std::make_shared<OrderInterpreter>();
    order_interpreter1_ = std::make_shared<OrderInterpreter>();
    target_subscriber_ =  this->create_subscription<ai_msgs::msg::PerceptionTargets>(
      sub_topic_,
      1,
      std::bind(&ActionImitationNode::subscription_callback,
      this,
      std::placeholders::_1)); 
    
    if (!msg_process_) {
        msg_process_ = std::make_shared<std::thread>(
            std::bind(&ActionImitationNode::MessageProcess, this));
    }
    order_interpreter_->control_serial_servo("stand");
    order_interpreter_->control_PWM_servo(1, pluse_, 400);
    order_interpreter_->control_PWM_servo(2, 1400, 400);
}
ActionImitationNode::~ActionImitationNode(){
  if (msg_process_ && msg_process_->joinable()) {
    process_stop_ = true;
    msg_process_->join();
    msg_process_ = nullptr;
  }
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
    if(num > POINT_FILTER_NUM){
        filter_result = angle_sum / (POINT_FILTER_NUM + 1);
        angle_sum = 0;
        num = 0;
    }
    num++;
}

void ActionImitationNode::angle_mean_filter1(const double& angle, int& num, std::vector<int>& angles, int& filter_result){

    angles[num % 6] = angle;
    if(num > 6){
        filter_result = (angles[0] + angles[1] + angles[2] + angles[3] + angles[4] + angles[5]) / 6;
    }
    num++;
}

void ActionImitationNode::subscription_callback(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg){
    int max_head_size = 0;
    ai_msgs::msg::Target max_head_target;
    bool start_control = false;
    {
        std::unique_lock<std::mutex> lock(target_mutex_);
        targets_msg_ = *targets_msg;
        update_data_ = true;
        lock.unlock();
    }

    {
        std::unique_lock<std::mutex> lock1(start_mutex_);
        start_control = start_control_;
    }
    if(start_control == true){
        std::cout<<"1"<<std::endl;
        for(const auto &target : targets_msg_.targets){
            if(target.rois[0].type == "head"){
                int size = target.rois[0].rect.height * target.rois[0].rect.width;
                if (size > max_head_size){
                    max_head_target = target;
                    max_head_size = size;
                }
            }
        }
        std::cout<<"2"<<std::endl;
        if(max_head_size == 0) return;
        static int p = 1400;
        int center_x = max_head_target.rois[0].rect.x_offset + max_head_target.rois[0].rect.width / 2;
        std::cout<<"3"<<std::endl;
        if(center_x > (320 + a)){
            p = int(p - (center_x - 320) * b); 
            

        } else if(center_x < (320 - a)) {
            p = int(p + (320 - center_x) * b); 
        }
        std::cout<<p<<std::endl;
        order_interpreter1_->control_PWM_servo(2, p, 100);
        
    }
    return;
}

void ActionImitationNode::MessageProcess(){
    while(process_stop_ == false){
        ai_msgs::msg::PerceptionTargets targets_msg;
        {
            std::unique_lock<std::mutex> lock(target_mutex_);
            if(update_data_ == true){
                targets_msg = targets_msg_;
            }
            lock.unlock();
        }

        ai_msgs::msg::Target max_hand_target;
        int max_hand_size = 0;
        for(const auto &target : targets_msg.targets){
            if(target.rois[0].type == "hand"){
                int size = target.rois[0].rect.height * target.rois[0].rect.width;
                if (size > max_hand_size){
                    max_hand_target = target;
                    max_hand_size = size;
                }
            }
        }

        if (max_hand_size == 0 || max_hand_target.attributes.size() == 0) continue;

        switch(int(max_hand_target.attributes[0].value)){
            //OK
            case 11:
                {
                    std::unique_lock<std::mutex> lock(start_mutex_);
                    start_control_ = true;
                }
                std::cout<<"start"<<std::endl;
                break;
            //good
            case 2:
            case 14:
                {
                    std::unique_lock<std::mutex> lock(start_mutex_);
                    start_control_ = false;
                }
                gesture_control_ = false;
                imitating_control_ = false;
                end_num_++;
                if(end_num_ > 5){
                    order_interpreter_->control_serial_servo("stand");
                    order_interpreter_->control_PWM_servo(2, 1400, 200);
                    std::cout<<"end"<<std::endl;
                }
                break;
        }

        if (start_control_ == false) continue;

        int max_body_size = 0;
        ai_msgs::msg::Target max_body_target;
        if(start_control_ == true){
            for(const auto &target : targets_msg.targets){
                if(target.rois[0].type == "body"){
                    int size = target.rois[0].rect.height * target.rois[0].rect.width;
                    if (size > max_body_size){
                        max_body_target = target;
                        max_body_size = size;
                    }
                }
            }
        }
        if(max_body_size == 0) continue;
        switch(int(max_hand_target.attributes[0].value)){
            //plam
            case 0:
            case 5:
                gesture_control_ = false;
                imitating_control_ = true;
                order_interpreter_->control_serial_servo(18, 500, 100);
                // std::cout<<"P"<<std::endl;
                break;
            default:
                gesture_control_ = true;
                imitating_control_ = false;
                order_interpreter_->control_serial_servo(18, 500, 100);
        }


        if(imitating_control_ == true){

            if(max_body_size == 0) continue;
            Point p8 (max_body_target.points[0].point[8].x, max_body_target.points[0].point[8].y);
            Point p6 (max_body_target.points[0].point[6].x, max_body_target.points[0].point[6].y);
            Point p12 (max_body_target.points[0].point[12].x, max_body_target.points[0].point[12].y);
            Point p10 (max_body_target.points[0].point[10].x, max_body_target.points[0].point[10].y);

            if(p10.x > p6.x) continue;
            double angle1 = angle_calculator(p8, p6, p12);
            // angle_mean_filter(angle1, num1_, angle_sum1_, filter_result1_);
            angle_mean_filter1(angle1, num1_, angles1, filter_result1_);
            int pluse1 = 850 - (750 / 180) * filter_result1_;
            order_interpreter_->control_serial_servo(7, pluse1, 0);


            double angle2 = angle_calculator(p6, p8, p10);
            if(angle2 == -1) angle2 = 180;

            // angle_mean_filter(angle2, num2_, angle_sum2_, filter_result2_);
            angle_mean_filter1(angle2, num2_, angles2, filter_result2_);
            double slope = double((p8.y - p6.y) )/ double((p8.x - p6.x));
            double val_y = slope * (p10.x - p6.x);
            int pluse2 = 0;
            if (val_y > (p10.y - p6.y)){
                pluse2 = 0 + (500 / 120) * (filter_result2_ - 60);
            } else {
                pluse2 = 900 - (400 / 120) * (filter_result2_ - 60);
            }
            order_interpreter_->control_serial_servo(6, pluse2, 0);
        }

        if(gesture_control_ == true){
            switch(int(max_hand_target.attributes[0].value)){
                
                case 3:
                    order_interpreter_->control_serial_servo(18, 900, 100);
                    break;
                //ThumbLeft
                case 12:
                    order_interpreter_->control_serial_servo("right_move_10");
                    break;
                //ThumbRight
                case 13:
                    order_interpreter_->control_serial_servo("left_move_10");
                    break;
            }
        }
    }
}


int main(int argc, char* argv[]){

    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ActionImitationNode>("ActionImitationNode"));

    rclcpp::shutdown();

    RCLCPP_WARN(rclcpp::get_logger("ActionImitationNode"), "Pkg exit");

    return 0;
}