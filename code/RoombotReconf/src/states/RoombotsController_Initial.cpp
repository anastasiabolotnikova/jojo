#include "RoombotsController_Initial.h"

#include "../RoombotsController.h"

void RoombotsController_Initial::configure(const mc_rtc::Configuration & config)
{
}

void RoombotsController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RoombotsController &>(ctl_);
}

bool RoombotsController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RoombotsController &>(ctl_);
  output("OK");
  return true;
}

void RoombotsController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RoombotsController &>(ctl_);
}

EXPORT_SINGLE_STATE("RoombotsController_Initial", RoombotsController_Initial)
