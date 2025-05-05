#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>

#include "api.h"
#include "ClosedLoopCollisionConstraint.h"

struct TeleoperationDemoArmAssist_DLLAPI TeleoperationDemoArmAssist : public mc_control::fsm::Controller
{
    TeleoperationDemoArmAssist(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;
private:
    mc_rtc::Configuration config_;
    double t_ = 0.0;

    std::vector<std::shared_ptr<clcoll::ClosedLoopCollisionConstraint>> collisionConstraints_;
};
