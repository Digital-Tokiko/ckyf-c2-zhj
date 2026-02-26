//
// Created by mizu on 2026/2/22.
//

//本程序最主要类，实现尝试根据匀加速直线运动模型的卡尔曼滤波器追踪目标
//尝试优先攻击被遮挡过的，再攻击瞄准过的，再攻击血量最低的，再攻击y值最大的敌方目标
//会尝试避开友军目标

#ifndef AUTOAIM_WS_RAWRECEIVER_H
#define AUTOAIM_WS_RAWRECEIVER_H

#include <set>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include "MyKalmanFilter.h"
#include "Fort.h"


enum type {
    RED,
    BLUE,
    NONE
};

class RawReceiver : public rclcpp::Node {
private:
    Fort fort_;
    rclcpp::TimerBase::SharedPtr aim_timer_;
    rclcpp::TimerBase::SharedPtr fire_timer_;

    /*struct MyKalmanInit cv_left_;
    struct MyKalmanInit cv_right_;*/
    struct MyKalmanInit ca_left_;
    struct MyKalmanInit ca_right_;

    /*cv::Mat A_cv_;
    cv::Mat P_cv_;
    cv::Mat H_cv_;
    cv::Mat Q_cv_;
    cv::Mat R_cv_;*/

    cv::Mat A_ca_;
    cv::Mat P_ca_;
    cv::Mat H_ca_;
    cv::Mat Q_ca_;
    cv::Mat R_ca_;

    /*cv::Mat x0_L_;
    cv::Mat x0_R_;*/
    cv::Mat x0_L_ca_;
    cv::Mat x0_R_ca_;

    cv::Mat zk_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_img_subscriber_;
    cv_bridge::CvImagePtr raw_img_;

    cv::Mat raw_img_mat_;
    cv::Mat grey_img_mat_;
    cv::Mat bg_mask_;

    std::vector<std::vector<cv::Point> > contours_;
    std::vector<cv::Rect> contours_rect_;

    int min_health_; //最低血量敌方目标的血量
    std::map<double, int> target_health_;
    int target_num_;
    std::map<int, double> target_index_; //用于存储进入场景次序
    std::map<double, MyKalmanFilter> kalman_filters_; //存储目标的滤波器
    std::map<double, MyKalmanFilter> avoid_kalman_filters_; //存储不需要打的目标的滤波器
    double last_aim_;
    std::vector<double> last_avoid_;
    std::set<double> grey_rects_; //用于判断要避开的目标是否是灰色，来改变避开的灵敏度

    cv::Point fort_point_;

    uint8_t type; //2代表未准备，0代表红色，1代表蓝色

    void Fire() {
        fort_.Fire();
        fire_timer_->cancel();
    }

    void CalcFire(); //计算发射角度并且发射！

    void ProcessInit();

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

public:
    RawReceiver(const std::string &fort_name,
                const std::string &node_name = "raw_img_receiver") : rclcpp::Node(node_name), fort_(fort_name, 115200) {
        /*A_cv_ = (cv::Mat_<double>(2, 2) << 1, 0.016, 0, 1);
        P_cv_ = cv::Mat::eye(2, 2, CV_64F);
        H_cv_ = (cv::Mat_<double>(1, 2) << 1, 0);
        Q_cv_ = (cv::Mat_<double>(2, 2) << 0.1, 0, 0, 0.2);
        R_cv_ = (cv::Mat_<double>(1, 1) << 1);*/

        A_ca_ = (cv::Mat_<double>(3, 3) << 1, 0.016, 0.016 * 0.016 / 2,
										   0, 1, 0.016,
										   0, 0, 1);
        P_ca_ = cv::Mat::eye(3, 3, CV_64F);
        H_ca_ = (cv::Mat_<double>(1, 3) << 1, 0, 0);
        Q_ca_ = (cv::Mat_<double>(3, 3) << 0.1, 0, 0,
										   0, 0.65, 0,
										   0, 0, 1.55);
        R_ca_ = (cv::Mat_<double>(1, 1) << 3.5);

        /*x0_L_ = (cv::Mat_<double>(2, 1) << 60, 210);
        x0_R_ = (cv::Mat_<double>(2, 1) << 1100, -210);*/
        x0_L_ca_ = (cv::Mat_<double>(3, 1) << 60,
											  200,
											  0);

        x0_R_ca_ = (cv::Mat_<double>(3, 1) << 1100,
											  -200,
											  -0);

        /*cv_left_ = MyKalmanInit{A_cv_ , P_cv_, H_cv_, Q_cv_, R_cv_, x0_L_};
        cv_right_ = MyKalmanInit{A_cv_ , P_cv_, H_cv_, Q_cv_, R_cv_, x0_R_};*/
        ca_left_ = MyKalmanInit{A_ca_, P_ca_, H_ca_, Q_ca_, R_ca_, x0_L_ca_};
        ca_right_ = MyKalmanInit{A_ca_, P_ca_, H_ca_, Q_ca_, R_ca_, x0_R_ca_};

        zk_ = cv::Mat::zeros(1, 1, CV_64F);

        raw_img_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10, std::bind(&RawReceiver::imageCallback, this, std::placeholders::_1));
        aim_timer_ = this->create_wall_timer(std::chrono::milliseconds(96), std::bind(&RawReceiver::CalcFire, this));

        last_aim_ = -1;

        fort_.TurnTo(90);

        type = NONE;
    }

    ~RawReceiver() = default;
};


#endif //AUTOAIM_WS_RAWRECEIVER_H
