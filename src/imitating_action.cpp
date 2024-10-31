#include <thread>
#include <chrono>

#include "order_interpreter.hpp"
#include "imitating_action.h"

#define POINT_FILTER_NUM 1


ActionImitationNode::ActionImitationNode(const std::string& node_name, const rclcpp::NodeOptions& options) : rclcpp::Node(node_name, options) {
    
    this->declare_parameter<std::string>("sub_topic", sub_topic_);
    this->declare_parameter<int>("offset", offset_);
    this->declare_parameter<int>("pluse", pluse_);
    this->declare_parameter<float>("ratio", ratio_);
    this->declare_parameter<int>("limit_right", limit_right_);
    this->declare_parameter<int>("limit_left", limit_left_);

    this->get_parameter<std::string>("sub_topic", sub_topic_);
    this->get_parameter<int>("offset", offset_);
    this->get_parameter<int>("pluse", pluse_);
    this->get_parameter<float>("ratio", ratio_);
    this->get_parameter<int>("limit_right", limit_right_);
    this->get_parameter<int>("limit_left", limit_left_);
    
    
    order_interpreter_ = std::make_shared<OrderInterpreter>();
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

bool ActionImitationNode::collision_detection(const double& degree1, const double& degree2){
    if(degree1 > 90) return false;
    double angle1 = degree1 * M_PI / 180.0;
    // double angle2 = degree2 * M_PI / 180.0;
    double sin1 = std::sin(angle1);
    float length1 = 2 + sin1 * 6;
    int degree = degree2 - (90 - degree1);
    double angle2 = degree * M_PI / 180.0;
    float length2 = 12 * std::cos(angle2);
    if (length2 > length1){
        return true;
    } else {
        return false;
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

// void ActionImitationNode::angle_mean_filter(const double& angle, int& num, int& angle_sum, int& filter_result){
//     angle_sum += angle;
//     if(num > POINT_FILTER_NUM){
//         filter_result = angle_sum / (POINT_FILTER_NUM + 1);
//         angle_sum = 0;
//         num = 0;
//     }
//     num++;
// }

void ActionImitationNode::angle_mean_filter(const double& angle, int& num, std::vector<int>& angles, int& filter_result){

    angles[num % 6] = angle;
    if(num > 6){
        filter_result = (angles[0] + angles[1] + angles[2] + angles[3] + angles[4] + angles[5]) / 6;
    }
    num++;
}

void ActionImitationNode::subscription_callback(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg){
    {
        std::unique_lock<std::mutex> lock(target_mutex_);
        targets_msg_ = *targets_msg;
        update_data_ = true;
        lock.unlock();
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
                update_data_ = false;
            } else {
                lock.unlock();
                continue;
            }
            lock.unlock();
        }
        int max_head_size = 0;
        ai_msgs::msg::Target max_head_target;
        if(start_control_ == true){
            for(const auto &target : targets_msg.targets){
                if(target.rois[0].type == "head"){
                    int size = target.rois[0].rect.height * target.rois[0].rect.width;
                    if (size > max_head_size){
                        max_head_target = target;
                        max_head_size = size;
                    }
                }
            }
            if(max_head_size == 0) continue;
            static int p = 1400;
            int center_x = max_head_target.rois[0].rect.x_offset + max_head_target.rois[0].rect.width / 2;
            if(center_x > (320 + offset_)){
                p = int(p - (center_x - 320) * ratio_); 
                order_interpreter_->control_PWM_servo(2, p, 100);
            } else if(center_x < (320 - offset_)) {
                p = int(p + (320 - center_x) * ratio_); 
                order_interpreter_->control_PWM_servo(2, p, 100);
            }
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
                    if(start_num_ > 3){
                        start_control_ = true;
                    }
                    start_num_++;
                }
                end_num_ = 0;

                std::cout<<"start"<<std::endl;
                break;
            //good
            case 2:
            case 14:
                end_num_++;
                start_num_ = 0;
                std::cout<<end_num_<<std::endl;
                if(end_num_ > 10){
                    order_interpreter_->control_serial_servo("stand");
                    order_interpreter_->control_serial_servo("bow");
                    order_interpreter_->control_serial_servo("stand");
                    order_interpreter_->control_PWM_servo(2, 1400, 200);
                    {
                        std::unique_lock<std::mutex> lock(start_mutex_);
                        start_control_ = false;
                    }
                    gesture_control_ = false;
                    imitating_control_ = false;
                    std::cout<<"end"<<std::endl;

                }
                break;
            default:
                start_num_ = 0;
                end_num_ = 0;

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
            //右臂移动
            {
                Point p8 (max_body_target.points[0].point[8].x, max_body_target.points[0].point[8].y);
                Point p6 (max_body_target.points[0].point[6].x, max_body_target.points[0].point[6].y);
                Point p12 (max_body_target.points[0].point[12].x, max_body_target.points[0].point[12].y);
                Point p10 (max_body_target.points[0].point[10].x, max_body_target.points[0].point[10].y);

                // if(p10.x > p6.x || p10.x > p12.x) continue;
                
                double angle1 = angle_calculator(p8, p6, p12);
                // angle_mean_filter(angle1, num1_, angle_sum1_, filter_result1_);
                angle_mean_filter(angle1, num1_, angles1, filter_result1_);
                int pluse1 = 900 - (750 / 180) * filter_result1_;
                order_interpreter_->control_serial_servo(7, pluse1, 0);


                double angle2 = angle_calculator(p6, p8, p10);


                if(angle2 == -1) angle2 = 180;

                // angle_mean_filter(angle2, num2_, angle_sum2_, filter_result2_);
                angle_mean_filter(angle2, num2_, angles2, filter_result2_);
                double slope = double((p8.y - p6.y) )/ double((p8.x - p6.x));
                double val_y = slope * (p10.x - p6.x);
                int pluse2 = 0;
                if (val_y > (p10.y - p6.y)){
                    pluse2 = 0 + (500.0 / 120) * (filter_result2_ - 60);
                    
                } else {
                    if(collision_detection(angle1, angle2) == true) {
                        std::cout<<"collision"<<std::endl;
                        continue;
                    }
                    pluse2 = 900 - (200.0 / 120) * (filter_result2_ - 60);
                }
                order_interpreter_->control_serial_servo(6, pluse2, 0);
            }


            //左臂移动
            {
                Point p11 (max_body_target.points[0].point[11].x, max_body_target.points[0].point[11].y);
                Point p5 (max_body_target.points[0].point[5].x, max_body_target.points[0].point[5].y);
                Point p7 (max_body_target.points[0].point[7].x, max_body_target.points[0].point[7].y);
                Point p9 (max_body_target.points[0].point[9].x, max_body_target.points[0].point[9].y);

                // if(p9.x < p5.x || p9.x < p11.x) continue;
                double angle3 = angle_calculator(p7, p5, p11);
                // angle_mean_filter(angle3, num3_, angle_sum3_, filter_result3_);
                angle_mean_filter(angle3, num3_, angles3, filter_result3_);
                int pluse3 = 100 + (750 / 180) * filter_result3_;
                order_interpreter_->control_serial_servo(15, pluse3, 0);


                double angle4 = angle_calculator(p5, p7, p9);

                if(angle4 == -1) angle4 = 180;

                // angle_mean_filter(angle4, num4_, angle_sum4_, filter_result4_);
                angle_mean_filter(angle4, num4_, angles4, filter_result4_);
                double slope = double((p7.y - p5.y) )/ double((p7.x - p5.x));
                double val_y = slope * (p9.x - p5.x);
                int pluse4 = 0;
                if (val_y > (p9.y - p5.y)){
                    pluse4 = 900 - (400.0 / 120) * (filter_result4_ - 60);
                    
                } else {
                    if(collision_detection(angle3, angle4) == true){
                        std::cout<<"collision"<<std::endl;
                        continue;
                    }
                    pluse4 = 0 + (500.0 / 120) * (filter_result4_ - 60);
                }
                order_interpreter_->control_serial_servo(14, pluse4, 0);
            }

        }

        if(gesture_control_ == true){
            switch(int(max_hand_target.attributes[0].value)){
                
                case 3:
                    left_control_num_ = 0;
                    right_control_num_ = 0;
                    order_interpreter_->control_serial_servo(18, 900, 100);
                    break;
                //ThumbLeft
                case 12:
                    right_control_num_++;
                    left_control_num_ = 0;
                    if (right_control_num_ > 2){
                        order_interpreter_->control_serial_servo("right_move_10");
                    }
                    break;
                //ThumbRight
                case 13:
                    left_control_num_++;
                    right_control_num_ = 0;
                    if(left_control_num_ > 2){
                        order_interpreter_->control_serial_servo("left_move_10");
                    }
                    break;
                default:
                    left_control_num_ = 0;
                    right_control_num_ = 0;
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