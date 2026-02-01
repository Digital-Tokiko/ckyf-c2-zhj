//
// Created by mizu on 2026/2/1.
//

#ifndef VIEWC2P1_COMMAND_H
#define VIEWC2P1_COMMAND_H
#include <cstdint>
#include <iostream>
#include <map>
#include <functional>

struct CommandLine {
    uint16_t time;
    uint8_t command;
    uint16_t argument1;
    uint16_t argument2;
    uint16_t argument3;
};

class Command {
private:
    uint16_t former_time_ = 0;
    struct CommandLine command_line_ = {0, 0, 0, 0, 0};
    static std::map<uint8_t, std::function<void(CommandLine &)> > command_map_;

    [[nodiscard]] std::function<void(CommandLine &)> GetFunc() const {
        return command_map_.find(command_line_.command)->second;
    }

public:
    Command() = default;

    ~Command() = default;

    static void RegisterFunction(uint8_t key, const std::function<void(CommandLine &)> &func) {
        command_map_.insert({key, func});
    }

    void UseCurrentCommand() {
        former_time_ = command_line_.time;
        GetFunc()(command_line_);
    }

    [[nodiscard]] uint16_t GetGapTime() const { return command_line_.time - former_time_; }

    friend std::istream &operator>>(std::istream &is, Command &cmd);
};


#endif //VIEWC2P1_COMMAND_H
