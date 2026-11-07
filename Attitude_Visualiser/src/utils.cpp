/** 
 * @file utils.cpp
 * @brief Implementation of utils.hpp        
 **/

#include "Attitude_Visualiser/utils.hpp"
#include "Attitude_Visualiser/serial_port.hpp"

#include <chrono>
#include <thread>
#include <vector>

namespace visualiser {

namespace {
/** =================================================
 *  File-local Helpers (not part of utils.hpp)
 ================================================= */
struct Edge { int a, b; }; // a and b are the indices of board_vertices that form the edge

// Wireframe representing the PCB with the IMU in BODY coordinates
// i.e. +X = forward, +Y = left, +Z = up
const std::vector<Vec3> board_vertices = {
    {  1.0,  0.6,  0.15}, {  1.0, -0.6,  0.15}, { -1.0, -0.6,  0.15}, { -1.0, 0.6,  0.15},
    {  1.0,  0.6, -0.15}, {  1.0, -0.6, -0.15}, { -1.0, -0.6, -0.15}, { -1.0, 0.6, -0.15},
};
const std::vector<Edge> board_edges = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0}, // top face
    {4, 5}, {5, 6}, {6, 7}, {7, 4}, // bottom face
    {0, 4}, {1, 5}, {2, 6}, {3, 7}, // verticals
}; 

/**
 * @brief Draws one 3D line segment on a 2D screen
 * 
 * @param renderer The SDL_Renderer rendering context used to track all rendering
 *                 settings. Can only render within SDL_Window.
 * @param proj The projector / camera model used to project points in the 3D world 
 *             frame onto the 2D screen 
 * @param a, b The two points forming the line segment in 3D world space
 */
void draw_world_line(SDL_Renderer *renderer, const Projector &proj, const Vec3 &a,
                     const Vec3 &b)
{
    double ax, ay, bx, by;

    // Convert 3D points to 2D screen coordinates
    if (!proj.project(a, ax, ay)) return;
    if (!proj.project(b, bx, by)) return;
    SDL_RenderLine(renderer, static_cast<float>(ax), static_cast<float>(ay),
                   static_cast<float>(bx), static_cast<float>(by));
}

} // unnamed namespace for internal linkage

/** =================================================
 *  Helper Functions for Data Sources
 ================================================== */
bool parse_quat_line(const std::string &line, Quat &out)
{
    if (line.compare(0, 5, "QUAT,") != 0) return false; // ensures the first 5 characters are "QUAT,"

    double w, x, y, z;
    int n = std::sscanf(line.c_str() + 5, "%lf,%lf,%lf,%lf,", &w, &x, &y, &z);
    if (n != 4) return false;   // sanity check that four values are filled
    
    out.w = w; out.x = x; out.y = y; out.z = z;
    out.normalise();
    return true;
}

void serial_reader_thread(std::string port, int baud, Shared_Orientation *shared,
                          std::atomic<bool> *running)
{
    // Create the local serial port
    Serial_Port serial;
    if (!serial.open(port, baud)) {
        std::fprintf(stderr, "Failed to open serial port '%s' @ %d baud.\n",
                     port.c_str(), baud);
        return;
    }
    std::fprintf(stderr, "Serial port '%s' is open @ %d baud. Yay!\n", port.c_str(), baud);

    std::string line;
    // Check whether the main app is running
    while (running->load()) {
        // Wait 200ms for a complete line
        if (serial.read_line_timeout(line, 200)) {
            Quat q;
            if (parse_quat_line(line, q)) {
                shared->set(q);
            } // parse and publish to the shared orientation
        }
    }
    std::fprintf(stderr, "Serial reader thread exiting (port closed or error).\n");
}

void demo_orientation_thread(Shared_Orientation *shared, std::atomic<bool> *running)
{
    double t = 0.0; // simulated clock, seconds
    while (running->load()) {
        // pseudo rotations based on time, radians
        double roll = 0.6 * std::sin(t * 0.7);
        double pitch = 0.4 * std::sin(t * 0.5 + 1.0);
        double yaw = 0.3 * t;

        // ZYX Euler to quaternion conversions
        double cr = std::cos(roll * 0.5),   sr = std::sin(roll * 0.5);
        double cp = std::cos(pitch * 0.5),  sp = std::sin(pitch * 0.5);
        double cy = std::cos(yaw * 0.5),    sy = std::sin(yaw * 0.5);

        Quat q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;
        q.normalise();
        shared->set(q);

        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // ~100 updates/sec to make it close to the real sensor's 104Hz rate
        t += 0.01;
    }
}

/** =================================================
 *  Helper Functions for Drawing
 ================================================== */
void draw_floor_grid(SDL_Renderer *renderer, const Projector &proj)
{
    SDL_SetRenderDrawColor(renderer, 60, 60, 66, 255);
    constexpr double extent = 4.0;
    constexpr double step = 0.5; // gridlines every 0.5 units
    constexpr double floor_z = -1.2; // negative height to be below the board

    // Note: Add +1e-9 to prevent floating point rounding errors from the skipping v=4 loop
    for (double v = -extent; v <= extent + 1e-9; v += step) {
        draw_world_line(renderer, proj, { v, -extent, floor_z }, { v, extent, floor_z });
        draw_world_line(renderer, proj, { -extent, v, floor_z }, { extent, v, floor_z });
    }
}

void draw_board_and_axes(SDL_Renderer *renderer, const Projector &proj, const Quat &q,
                         double staleness01)
{
    auto R = to_rotation_matrix(q);

    // Convert body frame vertices into the rotated corners in the world frame
    std::vector<Vec3> world_vertices;
    world_vertices.reserve(board_vertices.size());
    for (const auto &v : board_vertices) world_vertices.push_back(rotate(R, v));

    // Colour the board based on data freshness (cyan == fresh; grey == stale)
    Uint8 r = static_cast<Uint8>(60 + staleness01 * (120 - 60));
    Uint8 g = static_cast<Uint8>(220 - staleness01 * (220 - 120));
    Uint8 b = static_cast<Uint8>(220 - staleness01 * (220 - 120));
    SDL_SetRenderDrawColor(renderer, r, g, b, 255);
    for (const auto &e : board_edges) {
        draw_world_line(renderer, proj, world_vertices[e.a], world_vertices[e.b]);
    }

    // Draws the body frame's xyz axes to visualise which way they point as the
    // board rotates
    constexpr double axis_len = 1.6;
    Vec3 origin{ 0, 0, 0 };

    SDL_SetRenderDrawColor(renderer, 220, 60, 60, 255); // red for x-axis
    draw_world_line(renderer, proj, origin, rotate(R, Vec3{ axis_len, 0, 0 }));

    SDL_SetRenderDrawColor(renderer, 60, 220, 60, 255); // green for y-axis
    draw_world_line(renderer, proj, origin, rotate(R, Vec3{ 0, axis_len, 0 }));

    SDL_SetRenderDrawColor(renderer, 80, 120, 255, 255); // blue for z-axis
    draw_world_line(renderer, proj, origin, rotate(R, Vec3{ 0, 0, axis_len }));
}

void draw_rotation_hud(SDL_Renderer *renderer, Text_Renderer &text, const Quat &q, double age_sec,
                       float x, float y)
{
    if (!text.ready()) return; // no font so there is no HUD to draw =(

    // Compute the Euler angle representation of the orientation
    double roll, pitch, yaw;
    euler_from_quat(q, roll, pitch, yaw);

    // Compute the Axis-angle representation of the orientation
    double angle_rad, ax, ay, az;
    axis_angle_from_quat(q, angle_rad, ax, ay, az);

    constexpr double rad_to_deg = 180.0 / pi;
    const bool link_ok = age_sec < stale_after_sec;

    // HUD text formatting
    char buf[512];
    std::snprintf(buf, sizeof(buf),
                  "Euler (RPY, deg)\n"
                  "R:%7.2f P:%7.2f Y:%7.2f\n"
                  "\n"
                  "Quaternion (w x y z)\n"
                  "W:%6.3f X:%6.3f\n"
                  "Y:%6.3f Z:%6.3f\n"
                  "\n"
                  "Axis-angle (deg)\n"
                  "Axis:(%5.2f,%5.2f,%5.2f)\n"
                  "Angle:%7.2f",
                  roll * rad_to_deg, pitch * rad_to_deg, yaw * rad_to_deg,
                  q.w, q.x, q.y, q.z,
                  ax, ay, az, angle_rad * rad_to_deg);
    const std::string block(buf);
    const std::string link_text = link_ok ? "Link: Ok! :D" : "Link: Lost! D:";  // decide link status

    // Text measurements to determine the pixel size of the main block and the link line
    constexpr Text_Size size = Text_Size::Medium;
    const text_extent block_ext = text.measure(block, size);
    const text_extent link_ext = text.measure(link_text, size);
    const float gap = text.line_height(size);   // leave a blank line before the link status
    const float pad = 10.0f;

    // Drawing the HUD panel
    SDL_FRect panel {
        x - pad, y - pad,
        std::max(block_ext.width, link_ext.width) + 2.0f * pad,
        block_ext.height + gap + link_ext.height + 2.0f * pad,
    };
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer, 10, 10, 14, 170);
    SDL_RenderFillRect(renderer, &panel);

    text.draw(block, x, y, size, SDL_Color{210, 215, 225, 255}); // draw the main block text

    const SDL_Color link_colour = link_ok ? SDL_Color{60, 220, 60, 255} : SDL_Color{220, 60, 60, 255};
    text.draw(link_text, x, y + block_ext.height + gap, size, link_colour); // draw the link status text
}

} // visualiser namespace