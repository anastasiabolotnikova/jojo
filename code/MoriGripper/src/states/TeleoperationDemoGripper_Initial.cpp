#include "TeleoperationDemoGripper_Initial.h"

#include "../TeleoperationDemoGripper.h"

void TeleoperationDemoGripper_Initial::configure(const mc_rtc::Configuration & config)
{
}

void TeleoperationDemoGripper_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<TeleoperationDemoGripper &>(ctl_);
}

bool TeleoperationDemoGripper_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<TeleoperationDemoGripper &>(ctl_);
  output("OK");
  return true;
}

void TeleoperationDemoGripper_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<TeleoperationDemoGripper &>(ctl_);
}

EXPORT_SINGLE_STATE("TeleoperationDemoGripper_Initial", TeleoperationDemoGripper_Initial)
