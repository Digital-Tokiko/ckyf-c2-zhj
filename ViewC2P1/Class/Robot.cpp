//
// Created by mizu on 2026/1/31.
//

#include "Robot.h"
#include <iostream>

#include "RobotManager.h"

void Robot::Upgrade(const uint8_t to_level) {
    if (type_ == 0)
        switch (to_level) {
            case 1:
                SetMaxHealth(100);
                SetMaxHeat(100);
                break;
            case 2:
                SetMaxHealth(150);
                SetMaxHeat(200);
                break;
            case 3:
                SetMaxHealth(250);
                SetMaxHeat(300);
                break;
            default:
                std::cerr << "Invalid Level!" << std::endl;
        }
}

void Robot::TakeDamageForHeat(const uint16_t heat_change) {
    if (heat_change>GetHeat()) {
        if (GetHeat()>GetMaxHeat()) {
            TakeDamage(GetHeat()-GetMaxHeat());
        }
        SetHeat(0);
        return;
    }

    if (GetHeat()-heat_change < GetMaxHeat()) {
        TakeDamage(GetHeat()-GetMaxHeat());
    }
    else {
        TakeDamage(heat_change);
    }
    SetHeat(GetHeat()-heat_change);
}
