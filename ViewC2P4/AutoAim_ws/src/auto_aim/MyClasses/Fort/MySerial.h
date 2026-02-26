//
// Created by mizu on 2026/2/22.
//

#ifndef VIEWC2P4_MYSERIAL_H
#define VIEWC2P4_MYSERIAL_H

#include "serialPro/serialPro.h"

message_data head {
    uint8_t cmd_id = 0;
};

message_data tail{
};

class MySerial : public sp::serialPro<head, tail> {
private:
public:
    MySerial(const std::string &port, int baud) : serialPro(port, baud) {
    };

    ~MySerial() = default;
};


#endif //VIEWC2P4_MYSERIAL_H
