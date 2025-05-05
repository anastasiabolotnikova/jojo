#include "TeleoperationDemoTripod_Initial.h"

#include "../TeleoperationDemoTripod.h"

void TeleoperationDemoTripod_Initial::configure(const mc_rtc::Configuration & config)
{
}

void TeleoperationDemoTripod_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<TeleoperationDemoTripod &>(ctl_);
}

bool TeleoperationDemoTripod_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<TeleoperationDemoTripod &>(ctl_);
  output("OK");
  return true;
}

void TeleoperationDemoTripod_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<TeleoperationDemoTripod &>(ctl_);
}

EXPORT_SINGLE_STATE("TeleoperationDemoTripod_Initial", TeleoperationDemoTripod_Initial)
