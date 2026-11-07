/** 
 * @file  rpy_history.hpp
 * @brief A rolling-time window buffer that historises roll/pitch/yaw samples (in degrees).
 *        Used in the side-panel time-series graph.
 **/

#pragma once

#include <deque>

struct rpy_sample {
   double t_sec;       // seconds
   double roll_deg;
   double pitch_deg;
   double yaw_deg;
};

class Rpy_History {

public:
    explicit Rpy_History(double window_sec = 10.0) : window_sec_(window_sec) {}

    /**
     * @brief Appends a sample and drops anything older than the window_sec_.
     *        Assumes t_sec is non-decreasing across calls (which is why it is safe to trim
     *        readings only from the front of the buffer)
     * 
     * @param t_sec Timestamp of the current sample (in seconds).
     * @param roll_deg, pitch_deg, yaw_deg Roll, pitch, and yaw readings (in degrees) at the
     *                                     current time step.
     */
    void push(double t_sec, double roll_deg, double pitch_deg, double yaw_deg) {
        samples_.push_back({t_sec, roll_deg, pitch_deg, yaw_deg});
        while (!samples_.empty() && (t_sec - samples_.front().t_sec) > window_sec_) {
            samples_.pop_front();
        }
    }

    const std::deque<rpy_sample> &samples() const { return samples_; }
    double window_sec() const { return window_sec_; }

private:
    double window_sec_;
    std::deque<rpy_sample> samples_;
};

