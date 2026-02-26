//
// Created by mizu on 2026/2/22.
//

#include "RawReceiver.h"
#include <cmath>

void RawReceiver::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    static int hit_cnt;
    raw_img_ = cv_bridge::toCvCopy(msg);

    ProcessInit();

    grey_rects_.clear();

    if (type != NONE) cv::rectangle(grey_img_mat_, cv::Point(0, raw_img_mat_.rows - 100),
                                    cv::Point(raw_img_mat_.cols, raw_img_mat_.rows), cv::Scalar(0), -1);
    cv::findContours(grey_img_mat_, contours_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    //过滤掉背景数字造成的杂音
    for (auto &it: contours_) if (cv::boundingRect(it).area() > 150) contours_rect_.push_back(cv::boundingRect(it));

    bg_mask_ = cv::Mat::zeros(raw_img_mat_.size(), CV_8UC1);

    //cv::Mat test_mask_ = cv::Mat::zeros(raw_img_mat_.size() , CV_8UC3);

    cv::Mat hsv;
    cv::cvtColor(raw_img_mat_, hsv, cv::COLOR_BGR2HSV);

    std::vector<double> active_rects;
    std::map<double, cv::Rect> y_based_active_rects;
    std::vector<double> avoid_active_rects;
    std::map<double, cv::Rect> avoid_y_based_active_rects;

    if (type != NONE)
        for (auto &it: contours_rect_) {
            //遍历场上目标进行处理
            if (it.area() < 500) continue;
            it.x += 1;
            it.y += 3;
            it.width -= 1;
            it.height -= 4;

            bg_mask_ = cv::Mat::zeros(raw_img_mat_.size(), CV_8UC1);
            cv::rectangle(bg_mask_, it, cv::Scalar(255), -1);

            double y_center = it.y + it.height / 2;
            //因为用y坐标来区分目标，所以需要化归来防止被错误区分开
            for (auto &it1: kalman_filters_) {
                if (fabs(it1.first - y_center) < 10) {
                    y_center = it1.first;
                    break;
                }
            }

            for (auto &it2: avoid_kalman_filters_) {
                if (fabs(it2.first - y_center) < 10) {
                    y_center = it2.first;
                    break;
                }
            }

            double h_temp = cv::mean(hsv, bg_mask_)[0];
            double s_temp = cv::mean(hsv, bg_mask_)[1];
            if (h_temp > 4) {
                if (type == RED) {
                    if (it.x + it.width / 2 < 100) kalman_filters_.insert({y_center, MyKalmanFilter(ca_left_)});
                    else kalman_filters_.insert({y_center, MyKalmanFilter(ca_right_)});
                    target_health_.insert({y_center, 3});
                    active_rects.push_back(y_center);
                    y_based_active_rects.insert({y_center, it});
                    target_index_.insert({target_num_++, y_center});
                    //std::cout <<"BLUE Pos("<<it.x+it.width/2<<","<<y_center<<")";
                } else {
                    if (it.x + it.width / 2 < 100) avoid_kalman_filters_.insert({y_center, MyKalmanFilter(ca_left_)});
                    else avoid_kalman_filters_.insert({y_center, MyKalmanFilter(ca_right_)});
                    avoid_active_rects.push_back(y_center);
                    avoid_y_based_active_rects.insert({y_center, it});
                }
            } else if (s_temp > 1) {
                if (type == BLUE) {
                    if (it.x + it.width / 2 < 100) kalman_filters_.insert({y_center, MyKalmanFilter(ca_left_)});
                    else kalman_filters_.insert({y_center, MyKalmanFilter(ca_right_)});
                    target_health_.insert({y_center, 3});
                    active_rects.push_back(y_center);
                    y_based_active_rects.insert({y_center, it});
                    target_index_.insert({target_num_++, y_center});
                    //std::cout <<"RED Pos("<<it.x+it.width/2<<","<<y_center<<")";
                } else {
                    if (it.x + it.width / 2 < 100) avoid_kalman_filters_.insert({y_center, MyKalmanFilter(ca_left_)});
                    else avoid_kalman_filters_.insert({y_center, MyKalmanFilter(ca_right_)});
                    avoid_active_rects.push_back(y_center);
                    avoid_y_based_active_rects.insert({y_center, it});
                }
            } else if (fabs(y_center - last_aim_) > 20 && y_center != last_aim_) {
                //在上次打的目标下方的灰色目标
                if (y_center > last_aim_) {
                    grey_rects_.insert(y_center);
                    if (it.x + it.width / 2 < 100) avoid_kalman_filters_.insert({y_center, MyKalmanFilter(ca_left_)});
                    else avoid_kalman_filters_.insert({y_center, MyKalmanFilter(ca_right_)});
                    avoid_active_rects.push_back(y_center);
                    avoid_y_based_active_rects.insert({y_center, it});
                }
                //std::cout <<"GREY Pos("<<it.x+it.width/2<<","<<y_center<<")";
            } else {
                //打中后的目标变灰，防止丢失
                if (y_center == last_aim_) {
                    int hit_health = --target_health_.find(y_center)->second;
                    if (!hit_health <= 0) {
                        active_rects.push_back(y_center);
                        y_based_active_rects.insert({y_center, it});
                    } else {
                        target_health_.erase(y_center);
                        if (!target_index_.empty())
                            for (auto it1: target_index_) {
                                if (it1.second == y_center) {
                                    target_index_.erase(it1.first);
                                    break;
                                }
                            }
                    }
                }
            }
        }

    min_health_ = 3;
    for (auto it: target_health_) {
        if (it.second < min_health_) { min_health_ = it.second; }
    }

    for (auto it = kalman_filters_.begin(); it != kalman_filters_.end();) {
        //除去不需要的滤波对象，得系统学迭代器了***
        if (std::find(active_rects.begin(), active_rects.end(), it->first) == active_rects.end()) {
            it = kalman_filters_.erase(it);
        } else ++it;
    }

    for (auto it = avoid_kalman_filters_.begin(); it != avoid_kalman_filters_.end();) {
        //除去不需要的滤波对象，得系统学迭代器了***
        if (std::find(avoid_active_rects.begin(), avoid_active_rects.end(), it->first) == avoid_active_rects.end()) {
            it = avoid_kalman_filters_.erase(it);
        } else ++it;
    }
    //std::cout << std::endl;


    for (auto &pair: kalman_filters_) {
        pair.second.Predict();
        //std::cout << pair . second . GetXk_Expected() << " ";
        pair.second.Update((cv::Mat_<double>(1, 1) << y_based_active_rects[pair.first].x));
        //std::cout << pair . second . GetXk() << " ";
    }

    for (auto &pair: avoid_kalman_filters_) {
        pair.second.Predict();
        pair.second.Update((cv::Mat_<double>(1, 1) << avoid_y_based_active_rects[pair.first].x));
    }

    if (type == NONE) {
        //初始化识别炮塔颜色
        int max_area_index = 0;
        for (int i = 0; i < contours_rect_.size(); i++) if (
            contours_rect_[i].area() > contours_rect_[max_area_index].area()) max_area_index = i;

        contours_rect_[max_area_index].x += 15;
        contours_rect_[max_area_index].y += 15;
        contours_rect_[max_area_index].width /= 2;
        contours_rect_[max_area_index].height /= 2;
        cv::rectangle(bg_mask_, contours_rect_[max_area_index], cv::Scalar(255), -1);
        fort_point_ = cv::Point(contours_rect_[max_area_index].x + contours_rect_[max_area_index].width / 2,
                                contours_rect_[max_area_index].y + contours_rect_[max_area_index].height / 2);
        //cv::rectangle(test_mask_ , contours_rect_[max_area_index] , cv::Scalar(255,255,255) , -1);

        double h_temp = cv::mean(hsv, bg_mask_)[0];
        if (h_temp <= 10) {
            type = RED;
        } else type = BLUE;

        /*if (type == RED) {
            std::cout << "Red" << std::endl;
        }
        else {
            std::cout << "Blue" << std::endl;
        }*/
    }

    /*else {
        for ( auto &it : contours_rect_ ) {
            if ( it.area() < 100) continue;
            cv::rectangle(test_mask_ , it, cv::Scalar(255,255,255) , -1);
        }
    }*/

    //cv::bitwise_and(raw_img_mat_ , test_mask_, raw_img_mat_);

    /*cv::imshow("raw_img", raw_img_mat_);
    cv::waitKey(0);
    cv::destroyWindow("raw_img");*/
}

void RawReceiver::ProcessInit() {
    raw_img_mat_ = raw_img_->image;

    cv::cvtColor(raw_img_mat_, raw_img_mat_, cv::COLOR_RGB2BGR);
    cv::cvtColor(raw_img_mat_, grey_img_mat_, cv::COLOR_BGR2GRAY);

    contours_.clear();
    contours_rect_.clear();

    cv::threshold(grey_img_mat_, grey_img_mat_, 255, 0, 255);
    cv::GaussianBlur(grey_img_mat_, grey_img_mat_, cv::Size(3, 3), 1.0);
    cv::Canny(grey_img_mat_, grey_img_mat_, 100, 200, 3);
}

//实现了尽量盯着目标打，同时尝试避开右方单位并且在受遮挡的时候自动切换目标
void RawReceiver::CalcFire() {
    bool target_found = false;
    float angle;
    double y;
    std::map temp_filters = kalman_filters_;
    if (temp_filters.empty()) return;
    auto it = temp_filters.begin();

    while (!target_found) {
        if (temp_filters.empty()) return;

        if (!last_avoid_.empty()) {
            for (auto it2 = last_avoid_.begin(); it2 != last_avoid_.end(); ++it2) {
                it = temp_filters.find(*it2);
                if (it != temp_filters.end()) {
                    last_avoid_.erase(it2);
                    break;
                }
            }
        } else it = temp_filters.find(last_aim_);
        if (it == temp_filters.end()) {
            for (auto it1: target_health_) {
                if (it1.second == min_health_) {
                    it = temp_filters.find(it1.first);
                }
            }
            if (it == temp_filters.end()) it = temp_filters.find(target_index_.begin()->second);
            if (it == temp_filters.end()) it = std::prev(temp_filters.end());
        }

        if (it->second.GetFilterCnt() < 18 || fabs(it->second.GetXk().at<double>(1, 0)) < 80) {
            temp_filters.erase(it->first);
            continue;
        }

        cv::Mat Xk = it->second.GetXk();
        double x = Xk.at<double>(0, 0);
        double v = Xk.at<double>(1, 0);
        y = it->first;
        double x_predicted;
        double x_predicted_second;
        double time_gap;
        cv::Point point_predicted(x, y);
        time_gap = cv::norm(point_predicted - fort_point_) / 600;

        x_predicted = x + v * time_gap;
        point_predicted.x = x_predicted;
        time_gap = cv::norm(point_predicted - fort_point_) / 600;
        x_predicted_second = x + v * time_gap;
        x_predicted = (x_predicted_second + x_predicted) / 2;

        angle = atan2(fort_point_.y - y, x_predicted - fort_point_.x) * 180 / M_PI; //dy取了相反数

        if (avoid_kalman_filters_.empty()) {
            target_found = true;
            break;
        } else {
            auto it1 = avoid_kalman_filters_.begin();
            for (; it1 != avoid_kalman_filters_.end(); ++it1) {
                float angle1;
                cv::Mat Xk1 = it1->second.GetXk();
                double x1 = Xk1.at<double>(0, 0);
                double v1 = Xk1.at<double>(1, 0);
                double y1 = it1->first;
                double x_predicted1;
                double x_predicted_second1;
                double time_gap1;
                cv::Point point_predicted1(x, y);
                time_gap1 = cv::norm(point_predicted1 - fort_point_) / 600;

                x_predicted1 = x1 + v1 * time_gap1;
                point_predicted1.x = x_predicted1;
                time_gap1 = cv::norm(point_predicted1 - fort_point_) / 600;
                x_predicted_second1 = x1 + v1 * time_gap1;
                x_predicted1 = (x_predicted_second1 + x_predicted1) / 2;

                angle1 = atan2(fort_point_.y - y1, x_predicted1 - fort_point_.x) * 180 / M_PI; //dy取了相反数
                if (y1 > y) {
                    if (grey_rects_.count(y1)) {
                        if (fabs(angle1 - angle) < 15) break;
                    } else if (fabs(angle1 - angle) < 30 || (y1 > 450 && fabs(angle1 - angle) < 45) || (
                                   fabs(v1) < 80 && fabs(angle1 - angle) < 60)) break;
                } else if (fabs(angle1 - angle) < 40 * (fort_point_.y - y) / (fort_point_.y - y1)) break;
            }
            if (it1 == avoid_kalman_filters_.end()) { target_found = true; } else {
                last_avoid_.push_back(y);
                temp_filters.erase(y);
            }
        }
    }

    fort_.TurnTo(angle);
    last_aim_ = y;
    fire_timer_ = this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&RawReceiver::Fire, this));
}
