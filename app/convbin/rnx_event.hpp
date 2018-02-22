#pragma once
#include <nlohmann/json.hpp>
#include <iostream>
#include <stdexcept>
#include "rtklib.h"

using json = nlohmann::json;



class RinexEvent {
public:
    struct Marker {
        std::string name;
        std::string number;
        gtime_t gps_time;
    };
    enum class RtkState {KINEMATIC = 2, STATIC = 3};
    enum class EventPhase {KINEMATIC_PHASE = 2 , STATIC_PHASE = 3};
    RinexEvent(json&);
    const Marker& get_marker();
    gtime_t get_time();
    void set_time(gtime_t);
    int get_phase();
    int get_num_strings();
    bool is_kinematic();
    bool is_static();
private:
    RtkState state_;
    EventPhase event_phase_;
    Marker marker_;
};