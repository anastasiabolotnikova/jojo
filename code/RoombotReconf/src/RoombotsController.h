#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>

#include <Tasks/QPConstr.h>

#include "api.h"

struct RoombotsController_DLLAPI RoombotsController : public mc_control::fsm::Controller
{
    RoombotsController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;

    std::map<std::string, double>& blueJoJo() { return blueJoJo_; };
    std::map<std::string, double>& pinkJoJo() { return pinkJoJo_; };

private:
    mc_rtc::Configuration config_;

    double t_;

    mc_rtc::gui::ArrowConfig distanceArrowConfig;
    mc_rtc::gui::ArrowConfig safetyArrowConfig;
    mc_rtc::gui::PointConfig closestPointConfig;

    std::shared_ptr<tasks::qp::CollisionConstr> collConstr;
    int collId;

    Eigen::Vector3d midPoint_;
    Eigen::Vector3d safetyStartPoint_;
    Eigen::Vector3d safetyEndPoint_;

    std::map<std::string, double> blueJoJo_ = {{"M0", 0.}, {"M1", 0.}, {"M2", 0.},};
    std::map<std::string, double> pinkJoJo_ = {{"M0", 0.}, {"M1", 0.}, {"M2", 0.},};

    Eigen::Vector3d contactDistance_;

    bool roundJoJoTargets_ = false;

    bool reset16_ = false;
};
