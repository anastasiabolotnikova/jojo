#include "ListenToJoJo_PiX.h"

#include "../RoombotsController.h"

#include <mc_roombot/devices/JoJo.h>

void ListenToJoJo_PiX::configure(const mc_rtc::Configuration & config)
{
  config_.load(config);
}

void ListenToJoJo_PiX::start(mc_control::fsm::Controller & ctl_)
{
  if(!config_.has("robotMap")){
    mc_rtc::log::error_and_throw<std::runtime_error>("ListenToJoJo_PiX | robotMap config entry missing");
  }
  config_("robotMap", robotTopicJointsMap_);

  if(!config_.has("offsets")){
    mc_rtc::log::error_and_throw<std::runtime_error>("ListenToJoJo_PiX | offsets config entry missing");
  }
  config_("offsets", offsets_);

  for(auto & r : robotTopicJointsMap_){
    mc_rtc::log::success("Robot {}", r.first);

    if(ctl_.robots().robot(r.first).hasDevice<mc_roombot::JoJo>("JoJo"))
    {
      auto & jojo = ctl_.robots().robot(r.first).device<mc_roombot::JoJo>("JoJo");
      jojo.listenToJoJo(true, r.first, r.second, offsets_[r.first]);
    }
    else
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("No JoJo device on robot {}", r.first);
    }
  }
}

bool ListenToJoJo_PiX::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RoombotsController &>(ctl_);

  for(auto & r : robotTopicJointsMap_){
    std::string rN = r.first;
    auto & jojo = ctl_.robots().robot(rN).device<mc_roombot::JoJo>("JoJo");
    for(auto & t : jojo.correctedTarget()){
      std::string jN = t.first;
      double angle = t.second[0];

      // Save data for plots
      if(rN == "roombot_16"){
        ctl.blueJoJo()[jN] = angle;
      }else if (rN == "roombot_13"){
        ctl.pinkJoJo()[jN] = angle;
      }
    }
    // Update posture task target
    ctl_.getPostureTask(rN)->target(jojo.correctedTarget());
  }

  return false;
}

void ListenToJoJo_PiX::teardown(mc_control::fsm::Controller & ctl_)
{
  for(auto & r : robotTopicJointsMap_){
    auto & jojo = ctl_.robots().robot(r.first).device<mc_roombot::JoJo>("JoJo");
    jojo.listenToJoJo(false);
  }
}

EXPORT_SINGLE_STATE("ListenToJoJo_PiX", ListenToJoJo_PiX)
