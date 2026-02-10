//
// Created by mizu on 2026/2/9.
//

#ifndef VIEWP2C3_ELGAMALCLIENT_H
#define VIEWP2C3_ELGAMALCLIENT_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "rm_server/msg/get_el_gamal_params.hpp"
#include "rm_server/srv/el_gamal_encrypt.hpp"

class ElgamalClient : public rclcpp::Node{
    private:
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr result_publisher_;
    rclcpp::Subscription<rm_server::msg::GetElGamalParams>::SharedPtr params_subscriber_;
	rclcpp::Client<rm_server::srv::ElGamalEncrypt>::SharedPtr el_gamal_client_;
    uint64_t p;
    uint64_t a;
    uint64_t b;
	uint64_t n;
	uint64_t x;

    void ParamsCallback(const rm_server::msg::GetElGamalParams::SharedPtr msg);

	void ClientCallback(rclcpp::Client<rm_server::srv::ElGamalEncrypt>::SharedFuture client_future);
	
		void SendRequest() {
		auto request = std::make_shared<rm_server::srv::ElGamalEncrypt_Request>();
		request->public_key = b;

		el_gamal_client_->async_send_request(request, std::bind(&ElgamalClient::ClientCallback,this,std::placeholders::_1));
	}

    public:
    ElgamalClient(std::string node_name) : rclcpp::Node(node_name) {
        result_publisher_ = this->create_publisher<std_msgs::msg::Int64>("elgamal_result", 10);
        params_subscriber_ = this->create_subscription<rm_server::msg::GetElGamalParams>("elgamal_params",10,std::bind(&ElgamalClient::ParamsCallback,this,std::placeholders::_1));
		el_gamal_client_ = this->create_client<rm_server::srv::ElGamalEncrypt>("elgamal_service");
		n=1;
    }

    void SendResult(int64_t el_gamal_result) {
        std_msgs::msg::Int64 el_gamal_result_msg;
        el_gamal_result_msg.data = el_gamal_result;
        result_publisher_->publish(el_gamal_result_msg);
    }

	void SetN(int64_t n) {this->n = n;}
};


#endif //VIEWP2C3_ELGAMALCLIENT_H
