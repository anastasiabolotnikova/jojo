#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_solver/CoMIncPlaneConstr.h>

#include "api.h"
#include "ClosedLoopCollisionConstraint.h"

struct TeleoperationDemoTripod_DLLAPI TeleoperationDemoTripod : public mc_control::fsm::Controller
{
    TeleoperationDemoTripod(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;
private:
    mc_rtc::Configuration config_;
    std::vector<std::shared_ptr<clcoll::ClosedLoopCollisionConstraint>> collisionConstraints_;
    // Base position task
    std::shared_ptr<mc_tasks::EndEffectorTask> baseTask_;
    std::shared_ptr<mc_solver::CoMIncPlaneConstr> comIncPlaneConstraint_;

    //std::vector<std::string> tips = {"B_Tip_Y_Rot", "C_Tip_Y_Rot", "D_Tip_Y_Rot"};

    std::vector<Eigen::Vector3d> contactPoints_;
    std::vector<Eigen::Vector3d> marginPoints_;
    std::vector<Eigen::Vector3d> marginPointsInit_;

    std::vector<Eigen::Vector3d> midLinePoints_;
    std::vector<Eigen::Vector3d> normalEndPoints_;

    std::vector<Eigen::Vector3d> planeNormals_;
    std::vector<Eigen::Vector3d> planeNormalsPrev_;
    std::vector<Eigen::Vector3d> planeNormalsDot_;

    std::vector<Eigen::Vector3d> comProjections_;
    std::vector<Eigen::Vector3d> comProjectionsPrev_;
    std::vector<Eigen::Vector3d> comProjectionsSpeed_;

    std::vector<double> comDist2Planes_;

    Eigen::Vector3d com_;

    tasks::qp::ContactId nonSlidingContactId1_;
    tasks::qp::ContactId nonSlidingContactId2_;
    std::string nonSlidingContactSurface1_;
    std::string nonSlidingContactSurface2_;

    tasks::qp::ContactId getContactId(std::string s);

    bool useCoMCnstr_ = true;

    bool comPlanesSet_ = false;

    double off_;

    double t_ = 0.0;
};
