#include "rnx_event.hpp"


RinexEvent::RinexEvent(json &data) {

    try {
        state_ = static_cast<RtkState>(data["state"].get<int>());
        event_phase_ = static_cast<EventPhase>(data["state"].get<int>());

        if (state_ == RtkState::STATIC) {
            marker_.name = data["marker_name"].get<std::string>();
            marker_.number = data["marker_number"].get<std::string>();
        }
        if (!data["unix_timestamp"].is_null()) {
            double unix_time = data["unix_timestamp"].get<double>();
            gtime_t utc_time;
            utc_time.time = static_cast<time_t>(unix_time);
            utc_time.sec  = unix_time - static_cast<double>(utc_time.time);
            marker_.gps_time = utc2gpst(utc_time);
        }
        else {
           marker_.gps_time = gtime_t{0, 0};
        }
    }
    catch (json::exception& e)
    {
        throw std::runtime_error(e.what());
    }
}

const RinexEvent::Marker& RinexEvent::get_marker() {
    return marker_;
}

gtime_t RinexEvent::get_time() {
    return marker_.gps_time;
}

void RinexEvent::set_time(gtime_t time) {
    marker_.gps_time = time;
}

int RinexEvent::get_phase() {
    return static_cast<int>(event_phase_);
}

int RinexEvent::get_num_strings() {
    switch(state_) {
        case RtkState::KINEMATIC: return 2;
        case RtkState::STATIC:    return 3;
    }
}
bool RinexEvent::is_kinematic() {
    return state_ == RtkState::KINEMATIC;
}

bool RinexEvent::is_static() {
    return state_ == RtkState::STATIC;
}