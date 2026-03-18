#pragma once

#include "common_types.h"

namespace types{
    enum RobotName{
        M20 = 11,
    };

    enum RobotType{
        QuadrupedWheel = 1,
    };

    enum RobotMotionState{
        WaitingForStand = 0,
        StandingUp      = 1,
        JointDamping    = 2,
        RLControlMode   = 6,
        RLSensorControlMode = 7, // <--- 新增：带感知的运动模式
    };

    enum StateName{
        kInvalid      = -1,
        kIdle         = 0,
        kStandUp      = 1,
        kJointDamping = 2,
        kRLControl    = 6,
        kRLSensorControl = 7,    // <--- 新增：带感知的状态名称
    };

    enum RemoteCommandType{
        kKeyBoard = 0,
        kJoy = 1,
    };

    inline std::string GetAbsPath(){
        char buffer[PATH_MAX];
        if(getcwd(buffer, sizeof(buffer)) != NULL){
            return std::string(buffer);
        }
        return "";
    }
};