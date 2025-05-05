#include "TeleoperationDemoArmAssist_Initial.h"

#include "../TeleoperationDemoArmAssist.h"

void TeleoperationDemoArmAssist_Initial::configure(const mc_rtc::Configuration & config)
{
}

void TeleoperationDemoArmAssist_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<TeleoperationDemoArmAssist &>(ctl_);
}

bool TeleoperationDemoArmAssist_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<TeleoperationDemoArmAssist &>(ctl_);
  output("OK");
  return true;
}

void TeleoperationDemoArmAssist_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<TeleoperationDemoArmAssist &>(ctl_);
}

EXPORT_SINGLE_STATE("TeleoperationDemoArmAssist_Initial", TeleoperationDemoArmAssist_Initial)
