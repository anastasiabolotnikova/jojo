#include "ListenToJoJo.h"

#include "../TeleoperationDemoArmAssist.h"

#include <mc_mori/devices/JoJo.h>

void ListenToJoJo::configure(const mc_rtc::Configuration & config)
{
  config_.load(config);
}

void ListenToJoJo::start(mc_control::fsm::Controller & ctl_)
{
  if(!config_.has("topicMap")){
    mc_rtc::log::error_and_throw<std::runtime_error>("ListenToJoJo | topicMap config entry missing");
  }
  config_("topicMap", topicMap_);

  if(!config_.has("offsets")){
    mc_rtc::log::error_and_throw<std::runtime_error>("ListenToJoJo | offsets config entry missing");
  }
  config_("offsets", offsets_);

  if(ctl_.robot().hasDevice<mc_mori::JoJo>("JoJo"))
  {
    auto & jojo = ctl_.robot().device<mc_mori::JoJo>("JoJo");
    jojo.listenToJoJo(true, topicMap_, offsets_);
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No JoJo device");
  }
}

bool ListenToJoJo::run(mc_control::fsm::Controller & ctl_)
{
  auto & jojo = ctl_.robot().device<mc_mori::JoJo>("JoJo");
  ctl_.getPostureTask("mori_arm")->target(jojo.target());

  return false;
}

void ListenToJoJo::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & jojo = ctl_.robot().device<mc_mori::JoJo>("JoJo");
  jojo.listenToJoJo(false);
}

EXPORT_SINGLE_STATE("ListenToJoJo", ListenToJoJo)
