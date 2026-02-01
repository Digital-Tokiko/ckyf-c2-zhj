//
// Created by mizu on 2026/1/31.
//

#ifndef INFANTRYROBOT_H
#define INFANTRYROBOT_H
#include "Robot.h"


class InfantryRobot : public Robot {
private:

public:
    InfantryRobot(const uint16_t team_id, const uint16_t id) : Robot() {
        SetMaxHealth(100);
        SetMaxHeat(100);
        SetHeat(0);
        SetTeamId(team_id);
        SetId(id);
        SetLevel(1);
        SetType(0);
    };

    ~InfantryRobot() = default;
};


#endif // INFANTRYROBOT_H
