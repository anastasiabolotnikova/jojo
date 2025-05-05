#include "TeleoperationDemoArmAssist.h"
#include <mc_mori/devices/Led.h>
#include <mc_mori/devices/Debug.h>

using Color = mc_rtc::gui::Color;

TeleoperationDemoArmAssist::TeleoperationDemoArmAssist(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  config_.load(config);

  // Set leds of all modules to green
  for(auto & d: robot().devices())
  {
    if(d->type() == "Led")
    {
      auto & led = robot().device<mc_mori::Led>(d->name());
      led.command("led 0 15 0");
    }
  }

  // Add closed-loop collision constraints
  std::vector<mc_rbdyn::Collision> collisions = config("closedLoopCollision", std::vector<mc_rbdyn::Collision>{});
  for(auto & c : collisions){

    if(robot().hasBody(c.body1) && robot().hasBody(c.body2))
    {
      collisionConstraints_.push_back(std::make_shared<clcoll::ClosedLoopCollisionConstraint>(robots(), robots().robotIndex(), c.sDist, c.iDist, solver().dt()));
      collisionConstraints_[collisionConstraints_.size()-1]->setName("clcoll" + c.body1 + c.body2);
      collisionConstraints_[collisionConstraints_.size()-1]->setBodies(c.body1, c.body2);
      collisionConstraints_[collisionConstraints_.size()-1]->addToGUI(*gui());
      collisionConstraints_[collisionConstraints_.size()-1]->addToLogger(logger());

      solver().addConstraint(collisionConstraints_[collisionConstraints_.size()-1].get());
      solver().updateConstrSize();
    }
  }

  mc_rtc::log::success("TeleoperationDemoArmAssist init done");
}

bool TeleoperationDemoArmAssist::run()
{
  t_ += solver().dt();
  return mc_control::fsm::Controller::run(mc_solver::FeedbackType::Joints);
}

void TeleoperationDemoArmAssist::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);

  auto getMoriQ = [this]() -> const std::vector<double> &
  {
    static std::vector<double> q(robot().refJointOrder().size());
    auto & mbc = robot().mbc();
    const auto & rjo = robot().refJointOrder();
    int i=0;
    for(const auto & jn : rjo){
      for(auto & j : mbc.q[robot().jointIndexByName(jn)]){
        q[i] = j;
      }
      i++;
    }
    return q;
  };

  auto getMoriV = [this]() -> const std::vector<double> &
  {
    static std::vector<double> v(robot().refJointOrder().size());
    auto & mbc = robot().mbc();
    const auto & rjo = robot().refJointOrder();
    int i=0;
    for(const auto & jn : rjo){
      for(auto & j : mbc.alpha[robot().jointIndexByName(jn)]){
        v[i] = j;
      }
      i++;
    }
    return v;
  };

  auto getMoriT = [this]() -> const std::vector<double> &
  {
    static std::vector<double> t(robot().refJointOrder().size());
    auto & mbc = robot().mbc();
    const auto & rjo = robot().refJointOrder();
    int i=0;
    for(const auto & jn : rjo){
      for(auto & j : mbc.jointTorque[robot().jointIndexByName(jn)]){
        t[i] = j;
      }
      i++;
    }
    return t;
  };

  auto getMoriP = [this]() -> const std::vector<double> &
  {
    auto postureObjective = getPostureTask("mori_arm")->posture();
    static std::vector<double> p(robot().refJointOrder().size());
    auto & mbc = robot().mbc();
    const auto & rjo = robot().refJointOrder();
    int i=0;
    for(const auto & jn : rjo){
      auto ji = robot().jointIndexByName(jn);
      p[i] = postureObjective[ji][0];
      i++;
    }
    return p;
  };

  auto getMoriD = [this](unsigned int id) -> const std::vector<double> &
  {
    static std::vector<double> d(robot().refJointOrder().size());
    const auto & rjo = robot().refJointOrder();
    if(robot().hasDevice<mc_mori::Debug>("debug"))
    {
      auto & debugDevice = robot().device<mc_mori::Debug>("debug");
      int i=0;
      for(const auto & jn : rjo){
        d[i] = debugDevice.getDebug(jn, id);
        i++;
      }
    }
    return d;
  };

  auto getMoriA = [this]() -> const std::vector<double> &
  {
    static std::vector<double> a(robot().refJointOrder().size());
    auto & mbc = robot().mbc();
    const auto & rjo = robot().refJointOrder();
    int i=0;
    for(const auto & jn : rjo){
      for(auto & j : mbc.alphaD[robot().jointIndexByName(jn)]){
        a[i] = j;
      }
      i++;
    }
    return a;
  };

  logger().addLogEntry(
    "Mori_Joints", [getMoriQ]() -> const std::vector<double> & { return getMoriQ(); });

  logger().addLogEntry(
    "Mori_Speed", [getMoriV]() -> const std::vector<double> & { return getMoriV(); });

  logger().addLogEntry(
    "Mori_Torque", [getMoriT]() -> const std::vector<double> & { return getMoriT(); });

  logger().addLogEntry(
    "Mori_Target", [getMoriP]() -> const std::vector<double> & { return getMoriP(); });

  logger().addLogEntry(
    "Mori_Acceleration", [getMoriA]() -> const std::vector<double> & { return getMoriA(); });

  logger().addLogEntry(
    "Mori_Debug_RecievedCmd", [getMoriD]() -> const std::vector<double> & { return getMoriD(0); });

  logger().addLogEntry(
    "Mori_Debug_FilteredVel", [getMoriD]() -> const std::vector<double> & { return getMoriD(1); });

  logger().addLogEntry(
    "Mori_Debug_Position", [getMoriD]() -> const std::vector<double> & { return getMoriD(2); });

  logger().addLogEntry(
    "Mori_Debug_IntIntegralNeigh", [getMoriD]() -> const std::vector<double> & { return getMoriD(3); });
}
