//
// Created by mizu on 2026/2/23.
//

#ifndef AUTOAIM_WS_MYKALMANFILTER_H
#define AUTOAIM_WS_MYKALMANFILTER_H

#include <opencv2/opencv.hpp>

struct MyKalmanInit {
    cv::Mat A;
    cv::Mat P;
    cv::Mat H;
    cv::Mat Q;
    cv::Mat R;
    cv::Mat x0;
};

class MyKalmanFilter {
    //不是，卡尔曼滤波这么强的吗！
private:
    int filter_cnt_;

    cv::Mat x_k_;
    cv::Mat x_k_1_; //先前的状态
    cv::Mat K_k_;
    cv::Mat P_k_;
    cv::Mat P_k_1_;

    cv::Mat A_;
    cv::Mat B_;
    cv::Mat H_;
    cv::Mat Q_;
    cv::Mat R_;

    cv::Mat P_k_Expected_;
    cv::Mat x_k_Expected_;

public:
    const MyKalmanFilter &operator=(const MyKalmanFilter &other) {
        x_k_ = other.x_k_;
        x_k_1_ = other.x_k_1_;
        K_k_ = other.K_k_;
        P_k_ = other.P_k_;
        P_k_1_ = other.P_k_1_;
        A_ = other.A_;
        B_ = other.B_;
        H_ = other.H_;
        Q_ = other.Q_;
        R_ = other.R_;
        P_k_Expected_ = other.P_k_Expected_;
        x_k_Expected_ = other.x_k_Expected_;
        filter_cnt_ = other.filter_cnt_;
        return *this;
    }

    MyKalmanFilter(const struct MyKalmanInit &init) {
        filter_cnt_ = 0;
        x_k_1_ = init.x0;
        this->A_ = init.A;
        this->H_ = init.H;
        this->P_k_1_ = init.P;
        this->Q_ = init.Q;
        this->R_ = init.R;
    }

    MyKalmanFilter(const cv::Mat &x0, const cv::Mat &A, const cv::Mat &H, const cv::Mat &P, const cv::Mat &Q,
                   const cv::Mat &R) {
        filter_cnt_ = 0;
        x_k_1_ = x0;
        this->A_ = A;
        this->H_ = H;
        this->P_k_1_ = P;
        this->Q_ = Q;
        this->R_ = R;
    }

    MyKalmanFilter(const cv::Mat &x0, const cv::Mat &A, const cv::Mat &H, const cv::Mat &P, const cv::Mat &Q,
                   const cv::Mat &R,
                   const cv::Mat &B) {
        filter_cnt_ = 0;
        x_k_1_ = x0;
        this->A_ = A;
        this->H_ = H;
        this->P_k_1_ = P;
        this->Q_ = Q;
        this->R_ = R;
        this->B_ = B;
    }

    void Predict() {
        x_k_Expected_ = A_ * x_k_1_;
        P_k_Expected_ = A_ * P_k_1_ * A_.t() + Q_;
    }

    void Predict(const cv::Mat &u_k_1) {
        x_k_Expected_ = A_ * x_k_1_ + B_ * u_k_1;
        P_k_Expected_ = A_ * P_k_1_ * A_.t() + Q_;
    }

    void Update(const cv::Mat &z_k) {
        K_k_ = P_k_Expected_ * H_.t() / (H_ * P_k_Expected_ * H_.t() + R_);
        x_k_ = x_k_Expected_ + K_k_ * (z_k - H_ * x_k_Expected_);
        cv::Mat KH = K_k_ * H_;
        P_k_ = (cv::Mat::eye(KH.rows, KH.cols, CV_64F) - K_k_ * H_) * P_k_Expected_;

        x_k_1_ = x_k_;
        P_k_1_ = P_k_;
        filter_cnt_++;
    }

    cv::Mat GetXk_Expected() { return x_k_Expected_; }
    cv::Mat GetXk() { return x_k_; }
    int GetFilterCnt() { return filter_cnt_; }
};


#endif //AUTOAIM_WS_MYKALMANFILTER_H
