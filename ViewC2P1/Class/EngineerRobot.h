//
// Created by mizu on 2026/1/31.
//

#ifndef ENGINEERROBOT_H
#define ENGINEERROBOT_H
#include "Robot.h"


class EngineerRobot : public Robot {
private:

public:
    EngineerRobot(const uint16_t team_id, const uint16_t id) : Robot() {
        SetMaxHealth(300);
        SetMaxHeat(0);
        SetHeat(0);
        SetTeamId(team_id);
        SetId(id);
        SetLevel(1);
        SetType(1);
    }

    ~EngineerRobot() = default;
};


#endif //ENGINEERROBOT_H
