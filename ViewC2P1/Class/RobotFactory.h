//
// Created by mizu on 2026/1/31.
//

#ifndef VIEWC2P1_ROBOTFACTORY_H
#define VIEWC2P1_ROBOTFACTORY_H
#include "EngineerRobot.h"
#include "InfantryRobot.h"
#include <memory>


class RobotFactory {
public:
    RobotFactory() = delete;

    ~RobotFactory() = delete;

    static std::shared_ptr<Robot> MakeRobot(const uint8_t type, uint16_t team_id, uint16_t id) {
        switch (type) {
            case kInfantry:
                return std::make_shared<InfantryRobot>(team_id, id);
            case kEngineer:
                return std::make_shared<EngineerRobot>(team_id, id);
            default:
                return nullptr;
        }
    }
};


#endif //VIEWC2P1_ROBOTFACTORY_H
