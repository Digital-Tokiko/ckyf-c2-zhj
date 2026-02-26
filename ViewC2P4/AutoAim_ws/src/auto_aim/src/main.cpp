#include <iostream>
#include <string>
#include "RawReceiver.h"

int main() {
    std::string n;
    std::cout << "请输入串口名称：" << std::endl;
    std::cin >> n;

    rclcpp::init(0, nullptr);

    auto receiver_node = std::make_shared<RawReceiver>(n);

    rclcpp::spin(receiver_node);

    rclcpp::shutdown();

    return 0;
}
