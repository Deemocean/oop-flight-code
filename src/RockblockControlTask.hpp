#ifndef ROCKBLOCK_CONTROL_TASK_HPP_
#define ROCKBLOCK_CONTROL_TASK_HPP_

#include "sfr.hpp"
#include "constants.hpp"
#include <IridiumSBD.h>
#include <Arduino.h>


class RockblockControlTask{
    public:
        RockblockControlTask();
        void execute();
        IridiumSBD rockblock;
    private:
        int waiting_messages = -1;
        void handleCommand(int opcode, int argument);
        bool checkReady();
};

#endif