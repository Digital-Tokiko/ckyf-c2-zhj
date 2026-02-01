//
// Created by mizu on 2026/2/1.
//

#include "Command.h"

std::map<uint8_t, std::function<void(CommandLine &)> > Command::command_map_;

std::istream &operator>>(std::istream &is, Command &cmd) {
    CommandLine *cmdline = &cmd.command_line_;
    is >> cmdline->time >> cmdline->command >> cmdline->argument1 >> cmdline->argument2 >> cmdline->argument3;
    return is;
}
