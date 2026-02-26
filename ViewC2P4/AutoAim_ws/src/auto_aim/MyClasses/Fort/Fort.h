//
// Created by mizu on 2026/2/22.
//

#ifndef VIEWC2P4_FORT_H
#define VIEWC2P4_FORT_H

#include "MySerial.h"

message_data angle {
    float data;
};

class Fort {
private:
    MySerial serial_;

public:
    Fort(const std::string &port, int baud) : serial_(port, baud) {
    };

    ~Fort() = default;

    void TurnTo(float to_angle) { serial_.write(head{0x01}, angle{to_angle}, tail{}); }
    void Fire() { serial_.write(head{0x02}, "", tail{}); }
};

#endif //VIEWC2P4_FORT_H
