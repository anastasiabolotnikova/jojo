#include "RoombotsController.h"

#include <mc_roombot/devices/JoJo.h>

RoombotsController::RoombotsController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{

  distanceArrowConfig.color = mc_rtc::gui::Color(0, 0, 1, 0.5);
  distanceArrowConfig.end_point_scale = 0;
  distanceArrowConfig.head_diam = 0.005;
  distanceArrowConfig.head_len = 0.0001;
  distanceArrowConfig.scale = 1.;
  distanceArrowConfig.shaft_diam = 0.005;
  distanceArrowConfig.start_point_scale = 0;

  safetyArrowConfig.color = mc_rtc::gui::Color(1, 0, 0, 0.8);
  safetyArrowConfig.end_point_scale = 0;
  safetyArrowConfig.head_diam = 0.0053;
  safetyArrowConfig.head_len = 0.0001;
  safetyArrowConfig.scale = 1.;
  safetyArrowConfig.shaft_diam = 0.0053;
  safetyArrowConfig.start_point_scale = 0;

  closestPointConfig.color = mc_rtc::gui::Color(1, 0, 2, 0.5);
  closestPointConfig.scale = 0.01;

  t_ = 0.0;

  mc_rtc::log::success("RoombotsController init done");
}

bool RoombotsController::run()
{
  t_+=solver().dt();


  if(reset16_){
    sva::PTransformd X_0_s1 = robots().robot("roombot_13").surface("ACM1").X_0_s(robots().robot("roombot_13"));
    sva::PTransformd X_s2_b = robots().robot("roombot_16").surface("CP0").X_b_s().inv();
    sva::PTransformd X_b_r2 = robots().robot("roombot_16").X_b1_b2(robots().robot("roombot_16").surface("CP0").bodyName(), "base_link");
    robots().robot("roombot_16").posW(X_b_r2 * X_s2_b * X_0_s1);
    reset16_ = false;
  }

  // Update contact Distance variable
  if(robots().hasRobot("roombot_13") && robots().hasRobot("roombot_16")){
    contactDistance_ = (robots().robot("roombot_13").bodyPosW("acm1Frame").translation() - robots().robot("roombot_16").bodyPosW("cp0Frame").translation()).cwiseAbs();
  }

  //const Eigen::VectorXd & res = solver().solver.result();
  //mc_rtc::log::success("DV size {}", res.size());
  if(collision_constraints_.count({"roombot_16", "roombot_13"})){
    collConstr = collision_constraints_.at({"roombot_16", "roombot_13"})->collConstr;
    collId = 0; // ? How else to get it?
    midPoint_ = (collConstr->getCollisionData(collId).p1 + collConstr->getCollisionData(collId).p2) * 0.5;

    Eigen::Vector3d safetyUnit1 = (midPoint_ - collConstr->getCollisionData(collId).p1) / (collConstr->getCollisionData(collId).distance * 0.5);
    Eigen::Vector3d safetyUnit2 = (midPoint_ - collConstr->getCollisionData(collId).p2) / (collConstr->getCollisionData(collId).distance * 0.5);

    double safetyDistance = collision_constraints_.at({"roombot_16", "roombot_13"})->cols[0].sDist;

    safetyStartPoint_ = midPoint_ + safetyUnit1 * 0.5 * safetyDistance;
    safetyEndPoint_ = midPoint_ + safetyUnit2 * 0.5 * safetyDistance;
  }
  return mc_control::fsm::Controller::run();
}

void RoombotsController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);

  // Log distance between connecting plates
  if(robots().hasRobot("roombot_13") && robots().hasRobot("roombot_16")){
    contactDistance_ = (robots().robot("roombot_13").bodyPosW("acm1Frame").translation() - robots().robot("roombot_16").bodyPosW("cp0Frame").translation()).cwiseAbs();
  }

  // Add/Remove contact constraints
  if(robots().hasRobot("roombot_13") && robots().hasRobot("roombot_16")){
    gui()->addElement({"ACMContact", "StartRound"},
      mc_rtc::gui::Button("Start Round", [this]() {

        // Round posture task targets from JoJos to ensure successful ACM connection
        auto & jojo = robots().robot("roombot_13").device<mc_roombot::JoJo>("JoJo");
        jojo.roundJoJoTargets_ = true;
      })
    );

    gui()->addElement({"ACMContact", "Remove"},
      mc_rtc::gui::Button("Remove ACM contact", [this]() {
        removeContact({"roombot_13", "roombot_16", "ACM1", "CP0"});
        addContact({"roombot_16", "ground", "ACM0", "AllGround"});
      })
    );

    gui()->addElement({"ACMContact", "StopRound"},
      mc_rtc::gui::Button("Stop Rounding", [this]() {
        auto & jojo = robots().robot("roombot_13").device<mc_roombot::JoJo>("JoJo");
        jojo.roundJoJoTargets_ = false;
      })
    );

    gui()->addElement({"ACMContact", "Add Contact"},
      mc_rtc::gui::Button("Add Contact", [this]() {
        addContact({"roombot_13", "roombot_16", "ACM1", "CP0"});
      })
    );

    gui()->addElement({"ACMContact", "Reset16"},
      mc_rtc::gui::Button("Reset 16 posW", [this]() {

        removeContact({"roombot_16", "ground", "ACM0", "AllGround"});

        // Reset roombot_16 position in world frame to 'stick' to ACM1 of roombot_13
        sva::PTransformd X_0_s1 = robots().robot("roombot_13").surface("ACM1").X_0_s(robots().robot("roombot_13"));
        sva::PTransformd X_s2_b = robots().robot("roombot_16").surface("CP0").X_b_s().inv();
        sva::PTransformd X_b_r2 = robots().robot("roombot_16").X_b1_b2(robots().robot("roombot_16").surface("CP0").bodyName(), "base_link");
        robots().robot("roombot_16").posW(X_b_r2 * X_s2_b * X_0_s1);

        reset16_ = true;

      })
    );
  }

  gui()->addPlot("BlueJoJo",
              mc_rtc::gui::plot::X("Time (s)", [this]() { return t_; }),
              mc_rtc::gui::plot::Y("M0", [this]() { return blueJoJo_["M0"]; }, mc_rtc::gui::Color::Red),
              mc_rtc::gui::plot::Y("M1", [this]() { return blueJoJo_["M1"]; }, mc_rtc::gui::Color::Green),
              mc_rtc::gui::plot::Y("M2", [this]() { return blueJoJo_["M2"]; }, mc_rtc::gui::Color::Blue)
            );

  gui()->addPlot("PinkJoJo",
              mc_rtc::gui::plot::X("Time (s)", [this]() { return t_; }),
              mc_rtc::gui::plot::Y("M0", [this]() { return pinkJoJo_["M0"]; }, mc_rtc::gui::Color::Red),
              mc_rtc::gui::plot::Y("M1", [this]() { return pinkJoJo_["M1"]; }, mc_rtc::gui::Color::Green),
              mc_rtc::gui::plot::Y("M2", [this]() { return pinkJoJo_["M2"]; }, mc_rtc::gui::Color::Blue)
            );

  mc_rtc::log::info("{}", collision_constraints_.count({"roombot_16", "roombot_13"}));

  if(collision_constraints_.count({"roombot_16", "roombot_13"})){
    collConstr = collision_constraints_.at({"roombot_16", "roombot_13"})->collConstr;
    collId = 0; // ? How else to get it?
    gui()->addElement(
        {"Collision"},
        mc_rtc::gui::Arrow(
            "distance", distanceArrowConfig, [this]() -> const Eigen::Vector3d & { return collConstr->getCollisionData(collId).p1; },
            [this]() -> const Eigen::Vector3d & { return collConstr->getCollisionData(collId).p2; }),
      mc_rtc::gui::Point3D("p1", closestPointConfig, [this]() { return collConstr->getCollisionData(collId).p1; }),
      mc_rtc::gui::Point3D("p2", closestPointConfig, [this]() { return collConstr->getCollisionData(collId).p2; }),
      mc_rtc::gui::Arrow(
          "safetyDistance", safetyArrowConfig, [this]() -> const Eigen::Vector3d & { return safetyStartPoint_; },
          [this]() -> const Eigen::Vector3d & { return safetyEndPoint_; })
    );
  }

  auto getQ = [this](std::string robot) -> const std::vector<double> &
  {
    static std::vector<double> q(3);
    auto & mbc = robots().robot(robot).mbc();
    q[0] = mbc.q[robots().robot(robot).jointIndexByName("M0")][0];
    q[1] = mbc.q[robots().robot(robot).jointIndexByName("M1")][0];
    q[2] = mbc.q[robots().robot(robot).jointIndexByName("M2")][0];
    return q;
  };

  auto getV = [this](std::string robot) -> const std::vector<double> &
  {
    static std::vector<double> v(3);
    auto & mbc = robots().robot(robot).mbc();
    v[0] = mbc.alpha[robots().robot(robot).jointIndexByName("M0")][0];
    v[1] = mbc.alpha[robots().robot(robot).jointIndexByName("M1")][0];
    v[2] = mbc.alpha[robots().robot(robot).jointIndexByName("M2")][0];
    return v;
  };

  auto getBlueJoJo = [this]() -> const std::vector<double> &
  {
    static std::vector<double> q(3);
    q[0] = blueJoJo_["M0"];
    q[1] = blueJoJo_["M1"];
    q[2] = blueJoJo_["M2"];
    return q;
  };

  auto getPinkJoJo = [this]() -> const std::vector<double> &
  {
    static std::vector<double> q(3);
    q[0] = pinkJoJo_["M0"];
    q[1] = pinkJoJo_["M1"];
    q[2] = pinkJoJo_["M2"];
    return q;
  };

  auto getContactDistance = [this]() -> const Eigen::Vector3d &
  {
    return contactDistance_;
  };

  if(robots().hasRobot("roombot_16")){
    logger().addLogEntry(
      "roombot16_q", [getQ]() -> const std::vector<double> & { return getQ("roombot_16"); });

    logger().addLogEntry(
      "roombot16_dq", [getV]() -> const std::vector<double> & { return getV("roombot_16"); });

    logger().addLogEntry(
      "roombot16_jojo", [getBlueJoJo]() -> const std::vector<double> & { return getBlueJoJo(); });
  }

  if(robots().hasRobot("roombot_13")){
    logger().addLogEntry(
      "roombot13_q", [getQ]() -> const std::vector<double> & { return getQ("roombot_13"); });

    logger().addLogEntry(
      "roombot13_dq", [getV]() -> const std::vector<double> & { return getV("roombot_13"); });

    logger().addLogEntry(
      "roombot13_jojo", [getPinkJoJo]() -> const std::vector<double> & { return getPinkJoJo(); });
  }

  if(robots().hasRobot("roombot_13") && robots().hasRobot("roombot_16")){
    logger().addLogEntry(
      "contactDistance", [getContactDistance]() -> const Eigen::Vector3d & { return getContactDistance(); });
  }
}
