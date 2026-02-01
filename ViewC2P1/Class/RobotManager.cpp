//
// Created by mizu on 2026/1/31.
//

#include "RobotManager.h"

std::shared_ptr<Robot> RobotManager::SearchRobot(uint16_t team_id, uint16_t id, bool is_destroyed) {
    for (auto &it: robots_) {
        if (it->GetTeamId() == team_id && it->GetId() == id && it->IsDestroyed() == is_destroyed) {
            return it;
        }
    }
    return nullptr;
}

void RobotManager::CommandFunc_A(const CommandLine &command_line) {
    std::shared_ptr<Robot> robot = GetInstance().SearchRobot(command_line.argument1,
                                                             command_line.argument2, true);
    if (robot != nullptr) {
        if (robot->GetType() == command_line.argument3) {
            robot->SetHealth(robot->GetMaxHealth());
            robot->SetHeat(0);
            return;
        }
    }

    robot = GetInstance().SearchRobot(command_line.argument1, command_line.argument2, false);
    if (robot == nullptr) {
        std::shared_ptr<Robot> temp_robot = RobotFactory::MakeRobot(command_line.argument3, command_line.argument1,
                                                                    command_line.argument2);
        if (temp_robot) GetInstance().AddRobot(std::move(temp_robot));
        else std::cerr << "Make Robot Failed!" << std::endl;
    }
}

void RobotManager::CommandFunc_F(const CommandLine &command_line) {
    std::shared_ptr<Robot> robot = GetInstance().SearchRobot(command_line.argument1, command_line.argument2, false);

    if (robot != nullptr) {
        robot->TakeDamage(command_line.argument3);
        if (robot->IsDestroyed()) ShowDestroy(robot);
    }
}

void RobotManager::CommandFunc_H(const CommandLine &command_line) {
    std::shared_ptr<Robot> robot = GetInstance().SearchRobot(command_line.argument1, command_line.argument2, false);
    if (robot != nullptr) {
        if (robot->GetType() == kInfantry) {
            robot->SetHeat(robot->GetHeat() + command_line.argument3);
        }
    }
}

void RobotManager::CommandFunc_U(const CommandLine &command_line) {
    std::shared_ptr<Robot> robot = GetInstance().SearchRobot(command_line.argument1, command_line.argument2, false);
    if (robot != nullptr) {
        if (robot->GetType() == kInfantry && command_line.argument3 > robot->GetLevel()) {
            robot->Upgrade(command_line.argument3);
        }
    }
}
