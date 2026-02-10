//
// Created by mizu on 2026/2/10.
//

#include "ElgamalClient.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ElgamalClient>("elgamal_client");
    node -> SetN(23);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
