/** 
 * @file  shared_orientation.hpp
 * @brief Serves as a "mailbox" for the entire application with two key functions:
 *        1. Safely hands the latest orientation from the thread that receives it to the 
 *           thread that draws it.
 *        2. Records when the orientation arrives so that the visualiser knows if the
 *           orientation values have gone stale.
 * 
 *        This handoff implementation is required because the reader thread runs at ~100Hz
 *        (the read rate on the ESP32) while the main render loop runs at ~60FPS. The 
 *        Shared_Orientation class ensures the reader overwrites whatever the latest
 *        orientation is while the renderer reads whatever is available.
 * 
 *        Note:
 *        - There is no queue (i.e. intermediate values get overwritten), ensuring
 *          the latest pose is always rendered.
 *        - Using steady_clock instead of system_clock ensures that the time always 
 *          increases for measuring elapsed time.
 *        - Using mutex to lock also ensures that all four components of q are updated
 *          by the serial thread's write BEFORE the render thread reads q. 
 *        - std::lock_guard is intentionally used because on construction of its lock 
 *          object, it locks the mutex.  When the respective set() and get() function
 *          scope ends, the lock destructor automatically calls the mutex's unlock. Since
 *          C++ guarantees the destructors of local objects when their scope ends, the
 *          unlocking of the mutex with lock_guard is guaranteed too. This is important
 *          to avoid deadlock (i.e. permanent thread blocking).
 *          (See C++ RAII classes/wrappers for further information). 
 **/

#pragma once

#include <mutex>
#include <chrono>

#include "orientation_math.hpp"

class Shared_Orientation {

public:
    /**
     * @brief Used by the writer (serial thread). Called whenever a packet arrives.
     *        While locked, updates the quaternion and timestamp together so that the 
     *        reader never sees a quaternion paired with an old timestamap, and vice 
     *        versa.
     * 
     * @param q The latest orientation quaternion
     */
    void set(const Quat &q)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        q_ = q;
        last_update_ = std::chrono::steady_clock::now();
    }

    /**
     * @brief Used by the reader (render thread) to obtain the latest orientation
     *        quaternion and check for the elapsed time since the last set(). Called
     *        once per render frame.
     * 
     * @param age_sec Output reference parameter to store the elapsed time since
     *                the last set(); used to flag if the serial thread disconnects
     * 
     * @return A copy of q_ by value
     */
    Quat get(double &age_sec) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto now = std::chrono::steady_clock::now();
        age_sec = std::chrono::duration<double>(now - last_update_).count();
        return q_;
    }

private:
    mutable std::mutex mutex_;      // guards q_ and last_update_, declared mutable to allow itself to be modified in const functions
    Quat q_{1.0, 0.0, 0.0, 0.0};    // latest orientation
    std::chrono::steady_clock::time_point last_update_ = std::chrono::steady_clock::now();  // when q was last written
};