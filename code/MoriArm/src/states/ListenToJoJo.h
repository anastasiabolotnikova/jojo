#pragma once

#include <mc_control/fsm/State.h>
#include <thread>
#include "mqtt/async_client.h"


struct ListenToJoJo : mc_control::fsm::State
{

    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;

private:
    mc_rtc::Configuration config_;

    std::map<std::string, std::vector<std::string>> topicMap_;
    std::map<std::string, std::vector<double>> offsets_;
};
