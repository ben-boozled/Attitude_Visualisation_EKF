/** 
 * @file utils.hpp
 * @brief A collection of constexpr and helper functions to parse quaternion data, 
 *        draw the board and scene, and render the HUD.
 **/

#include <SDL3/SDL.h>
#include <atomic>
#include <string>

#include "Attitude_Visualiser/orientation_math.hpp"
#include "Attitude_Visualiser/shared_orientation.hpp"
#include "Attitude_Visualiser/text_renderer.hpp"

#pragma once

namespace visualiser {

/** ============================================
 *  Constants
 ============================================ */
inline constexpr int viewport_w = 1450; //900
inline constexpr int graph_panel_w = 540; //340
inline constexpr int window_w = viewport_w + graph_panel_w; // initial window width
inline constexpr int window_h = 950; // 650                       // initial window height

inline constexpr double stale_after_sec = 1.0;          // fades to grey if no data for this duration
inline constexpr double disconnected_after_sec = 3.0;   // treat link as lost
inline constexpr double graph_window_sec = 10.0;        // RPY history duration for graph panel

inline constexpr double pi = 3.1415926535897932384;

/** ============================================
 *  Helper Functions for Data Sources
 ============================================ */

/**
 * @brief Parses a line of the form "QUAT,<w>,<x>,<y>,<z>". Turns that line
 *        into a Quat datatype.
 * 
 * @param line The line to be parsed
 * @param out Output reference parameter to write the Quat output; the out is only
 *            written on success to avoid corrupting the caller's quaternion
 * 
 * @return false if the line does not match the "QUAT,<w>,<x>,<y>,<z>" format,
 *         true on a successful parse
 */
bool parse_quat_line(const std::string &line, Quat &out);

/**
 * @brief Creates a local serial port (so that the destructor automatically closes
 *        the port when the function returns).
 * 
 * @param port Device name e.g. "COM3" or "/dev/ttyUSB0" (pass by value so that
 *             every data thread owns its own copy without pointing at something
 *             that has already been destroyed)
 * @param baud Serial speed
 * @param shared Output reference parameter for the updated shared quaternion
 * @param running Shutdown flag owned by the main() app function (using std:atomic
 *                instead of std::mutex because there is only a single variable here 
 *                to protect; explicitly tells the compiler to perform every read and
 *                write for this running status flag)
 */
void serial_reader_thread(std::string port, int baud, Shared_Orientation *shared,
                          std::atomic<bool> *running);

/**
 * @brief A demo thread used to test the visualiser without any hardware. Feeds a
 *        pseudo-evolving rotation into the visualiser.
 * 
 * @param shared Output reference parameter for the updated shared quaternion
 * @param running Shutdown flag owned by the main() app function (using std:atomic
 *                instead of std::mutex because there is only a single variable here 
 *                to protect; explicitly tells the compiler to perform every read and
 *                write for this running status flag)
 */
void demo_orientation_thread(Shared_Orientation *shared, std::atomic<bool> *running);

/** ============================================
 *  Helper Functions for Drawing
 ============================================ */

/**
 * @brief Draws the floor grid below the attitude visualiser board model. The grid
 *        spans +- 4 units in the X and Y directions. This grid acts as a fixed
 *        reference while the simulated board model rotates.
 * 
 * @param renderer The SDL_Renderer rendering context used to track all rendering
 *                 settings. Can only render within SDL_Window.
 * @param proj The projector / camera model used to project points in the 3D world 
 *             frame onto the 2D screen. 
 */
void draw_floor_grid(SDL_Renderer *renderer, const Projector &proj);

/**
 * @brief Draws the wireframe representing the IMU board in the orientation described
 *        by the given quaternion.
 * 
 * @param renderer The SDL_Renderer rendering context used to track all rendering
 *                 settings. Can only render within SDL_Window.
 * @param proj The projector / camera model used to project points in the 3D world 
 *             frame onto the 2D screen. 
 * @param q The quaternion representing the orientation that converts vectors in the
 *          IMU board's body frame to the world frame.
 * @param staleness01 Determines the board colour based on data freshness; 0 for fresh
 *                    data, 1 for stale data
 */
void draw_board_and_axes(SDL_Renderer *renderer, const Projector &proj, const Quat &q,
                         double staleness01);

/**
 * @brief Draws the top-left panel on the visualiser to display the orientation of the
 *        IMU (in quaternions, Euler angles, and axis-angle), and the serial link status 
 *        to the physical IMU.
 * 
 * @param renderer The SDL_Renderer rendering context used to track all rendering
 *                 settings. Can only render within SDL_Window.
 * @param text The project's text drawing helper object
 * @param q The quaternion representing the orientation that converts vectors in the
 *          IMU board's body frame to the world frame.
 * @param age_sec Time since the orientation was last updated, in seconds
 * @param x, y Screen coordinates of the top-left corner of the text
 */
void draw_rotation_hud(SDL_Renderer *renderer, Text_Renderer &text, const Quat &q, double age_sec,
                       float x, float y);

} // visualiser namespace