//
// Created by mizu on 2026/1/31.
//

#ifndef ROBOTMANAGER_H
#define ROBOTMANAGER_H
#include "Robot.h"
#include "Command.h"
#include "RobotFactory.h"
#include <iostream>
#include <memory>
#include <vector>

class RobotManager {
private:
    std::vector<std::shared_ptr<Robot> > robots_;
    Command command_;

    static void RegisterFunction(const uint8_t key, const std::function<void(CommandLine&)> &func) {
        Command::RegisterFunction(key, func);
    }

    std::shared_ptr<Robot> SearchRobot(uint16_t team_id, uint16_t id,bool is_destroyed);

    void AddRobot(std::shared_ptr<Robot> new_robot) { robots_.push_back(std::move(new_robot)); }

    static void ShowDestroy(const std::shared_ptr<Robot> &robot){std::cout<<"D "<<robot->GetTeamId()<<" "<<robot->GetId()<<std::endl;}

    RobotManager() {
        RegisterFunction('A',CommandFunc_A);
        RegisterFunction('F',CommandFunc_F);
        RegisterFunction('U',CommandFunc_U);
        RegisterFunction('H',CommandFunc_H);
    };

public:
    void ReadCommandAndUse() {
        std::cin >> command_;
        for (auto& it: robots_) {
            if (it->GetType()==kInfantry && !it->IsDestroyed()) {
                it->TakeDamageForHeat(command_.GetGapTime());
                if (it->IsDestroyed()) ShowDestroy(it);
            }
        }
        command_.UseCurrentCommand();
    }

    static RobotManager &GetInstance() {
        static RobotManager instance;
        return instance;
    }

    static void CommandFunc_A(const CommandLine &command_line);
    static void CommandFunc_F(const CommandLine &command_line);
    static void CommandFunc_H(const CommandLine &command_line);
    static void CommandFunc_U(const CommandLine &command_line);
};


#endif //ROBOTMANAGER_H
