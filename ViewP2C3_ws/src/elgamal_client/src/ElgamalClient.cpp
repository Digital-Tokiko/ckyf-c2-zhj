//
// Created by mizu on 2026/2/9.
//

#include "ElgamalClient.h"

/*uint64_t mod_pow(uint64_t a,uint64_t n,uint64_t p){
    uint64_t result=1;
    a%=p;
    for (uint64_t i=0;i<n;i++){
        result=(result*a)%p;
    }
    return result%p;
}*/

//借ai了解到模逆元，费马小定理和快速幂,不过此处用快速幂即可
uint64_t mod_pow(int64_t a,int64_t n,int64_t p){
    uint64_t result=1;
    while(n > 0){
    	if ( n & 1) result = result *a %p;
    	a = a*a %p;
    	n>>=1;
    }
    return result;
}

void ElgamalClient::ClientCallback(rclcpp::Client<rm_server::srv::ElGamalEncrypt>::SharedFuture client_future) {
    auto response = client_future.get();
    uint64_t y1=response->y1;
    uint64_t y2=response->y2;
    uint64_t temp=mod_pow(y1,n*(p-2),p);
    temp*=y2;
    x=temp%p;
//    RCLCPP_INFO(this->get_logger(), "得到y1,y2：%d %d.", int(y1),int(y2));
//    RCLCPP_INFO(this->get_logger(), "发送结果：%d.", int(x));
    SendResult(x);
}

void ElgamalClient::ParamsCallback(const rm_server::msg::GetElGamalParams::SharedPtr msg) {
    a=msg->a;
    p=msg->p;
    b=mod_pow(a,n,p);
//    RCLCPP_INFO(this->get_logger(), "接收到a,p，得到b：%d %d %d.", int(a),int(p),int(b));
    SendRequest();
}
