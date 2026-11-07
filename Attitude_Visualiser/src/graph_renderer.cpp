/** 
 * @file graph_renderer.cpp
 * @brief Implementation of graph_renderer.hpp        
 **/

#include <algorithm>
#include <cstdio>

#include "Attitude_Visualiser/graph_renderer.hpp"

namespace {

constexpr double y_axis_min_deg = -180.0;
constexpr double y_axis_max_deg = 180.0;
constexpr Text_Size label_size = Text_Size::Small;

/* Colour settings */
// roll=reg, pitch=green, yaw=blue (match the body-axis RGB convention in main.cpp)
constexpr SDL_Color roll_colour      {220, 60, 60, 255};
constexpr SDL_Color pitch_colour     {60, 220, 60, 255};
constexpr SDL_Color yaw_colour       {80, 140, 255, 255};
constexpr SDL_Color grid_colour      {55, 55, 62, 255};
constexpr SDL_Color axis_text_colour {150, 150, 160, 255};
constexpr SDL_Color panel_bg_colour  {10, 10, 14, 255};

/**
 * @brief Maps degree values to a its y-coordinate on the rpy plot.
 * 
 * @param deg The roll/pitch/yaw in degrees
 * @param plot The struct used to store the rpy plot's properties
 * 
 * @return The y-coordinate of the deg value on the rpy plot
 */
float deg_to_screen_y(double deg, const Graph_Rect &plot)
{
    double t = (deg - y_axis_min_deg) / (y_axis_max_deg - y_axis_min_deg); // 0 at min, 1 at max
    t = std::clamp(t, 0.0, 1.0);
    return static_cast<float> (plot.y + plot.h * (1.0 - t)); // screen +Y points downwards
}

/**
 * @brief Maps the timestamp of the orientation reading to its x-coordinate on
 *        the rpy plot. x=0 and x=plot.w form the left and right edges of the
 *        plot respectively.
 * 
 * @param age_sec The timestamp of the reading in seconds
 * @param window_sec The length of visible timespan (in seconds) for the plot from
 *                   its left to right edges.
 * @param plot The struct used to store the rpy plot's properties
 * 
 * @return The x-coordinate of the deg value on the rpy plot
 */
float age_to_screen_x(double age_sec, double window_sec, const Graph_Rect &plot)
{
    double frac_from_right = std::clamp(age_sec / window_sec, 0.0, 1.0);
    return static_cast<float> (plot.x + plot.w * (1.0 - frac_from_right)); 
}

/**
 * @brief Draws a single line on a horizontally scrolling graph (representing roll, pitch
 *        or yaw over time).
 * 
 * @param renderer The SDL_Renderer rendering context used to track all rendering
 *                 settings. Can only render within SDL_Window.
 * @param history The object storing the recent samples of roll/pitch/yaw readings and
 *                their timestamp
 * @param plot The struct used to store the rpy plot's properties
 * @param member A data member (roll_deg, pitch_deg, yaw_deg) of the rpy_sample
 * @param colour The draw colour (r,g,b,a)
 */
void draw_series(SDL_Renderer *renderer, const Rpy_History &history, const Graph_Rect &plot,
                 double rpy_sample::*member, SDL_Color colour)
{
    const auto &samples = history.samples();
    if (samples.size() < 2) return; // unable to draw line with <2 points

    SDL_SetRenderDrawColor(renderer, colour.r, colour.g, colour.b, colour.a);

    const double t_latest = samples.back().t_sec;   // take the newest sample timestamp as the "now" reference
    const double window = history.window_sec();

    bool have_prev = false;
    float prev_x = 0.0f, prev_y = 0.0f;

    for (const auto &s : samples) {
        /* Map the data to pixel positions */
        float x = age_to_screen_x(t_latest - s.t_sec, window, plot);
        float y = deg_to_screen_y(s.*member, plot);
        if (have_prev) {
            SDL_RenderLine(renderer, prev_x, prev_y, x, y);
        } // draws a line from the previous point to the current point
        prev_x = x;
        prev_y = y;
        have_prev = true;
    }
}

/**
 * @brief Sets the draw colour for the renderer
 * 
 * @param renderer The SDL_Renderer rendering context used to track all rendering
 *                 settings. Can only render within SDL_Window.
 * @param c The draw colour (r,g,b,a)
 */
void set_colour(SDL_Renderer *renderer, SDL_Color c)
{
    SDL_SetRenderDrawColor(renderer, c.r, c.g, c.b, c.a);
}

}

void draw_rpy_graph(SDL_Renderer *renderer, Text_Renderer &text, const Rpy_History &history, 
                    const Graph_Rect &rect)
{
    /* Colour the background panel */
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    set_colour(renderer, panel_bg_colour);
    SDL_FRect bg {rect.x, rect.y, rect.w, rect.h};
    SDL_RenderFillRect(renderer, &bg);

    /* Derive the layout from the measured text */
    constexpr float padding = 8.0f;                                // standard margin in px
    const float line_h = text.line_height(label_size);
    const float label_w = text.measure("-180", label_size).width;  // width is based on "-180" which is the widest label
    const float left_axis_w = padding + label_w + 6.0f;            // horizontal space reserved for labels

    text.draw("Roll / Pitch / Yaw (deg)", rect.x + padding, rect.y + padding, label_size,
              axis_text_colour); 

    /* Draw the legend */
    float legend_x = rect.x + padding;
    const float legend_y = rect.y + padding + line_h;
    const struct { const char *name; SDL_Color colour; } legend[] = {
        {"Roll", roll_colour}, {"Pitch", pitch_colour}, {"Yaw", yaw_colour},
    };
    for (const auto &entry : legend) {
        text_extent e = text.draw(entry.name, legend_x, legend_y, label_size, entry.colour);
        legend_x += e.width + 14.0f; // lay out the legend names from left to right
    }

    /* Compute the inner plot rectangle */
    const float top = padding + 2.0f * line_h + padding + 0.5f * line_h;
    const float bottom = 0.5f * line_h + padding + line_h + padding;
    Graph_Rect plot{
        rect.x + left_axis_w,
        rect.y + top,
        rect.w - left_axis_w - padding,
        rect.h - top - bottom,
    };
    if (plot.w <= 0.0f || plot.h <= 0.0f) return; // bails out if the panel is too small (to avoid drawing with negative dimensions)

    /* Gridlines + right-aligned Y-axis labels (vertically centred on each line)*/
    for (double deg : {-180.0, -90.0, 0.0, 90.0, 180.0}) {
        float y = deg_to_screen_y(deg, plot);
        set_colour(renderer, grid_colour);
        SDL_RenderLine(renderer, plot.x, y, plot.x + plot.w, y); // draw a horizontal line for each deg

        char label[8];
        std::snprintf(label, sizeof(label), "%d", static_cast<int>(deg));
        float lw = text.measure(label, label_size).width;
        text.draw(label, plot.x - 6.0f - lw, y - 0.5f * line_h, label_size, axis_text_colour);
    }

    /* Plot the 3x time series (drawn after gridlines to render on top) */
    draw_series(renderer, history, plot, &rpy_sample::roll_deg, roll_colour);
    draw_series(renderer, history, plot, &rpy_sample::pitch_deg, pitch_colour);    
    draw_series(renderer, history, plot, &rpy_sample::yaw_deg, yaw_colour);

    /* Caption (to print the window period of the historical data)*/
    char caption[32];
    std::snprintf(caption, sizeof(caption), "Last %ds", static_cast<int>(history.window_sec()));
    text.draw(caption, plot.x, rect.y + rect.h - padding - line_h, label_size, axis_text_colour);
}