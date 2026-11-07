/** 
 * @file graph_renderer.hpp
 * @brief Draws a time series line graph of roll/pitch/yaw (in degrees) from
 *        rpy_history. Uses Text_Renderer to draw labels.
 **/

#pragma once

#include <SDL3/SDL.h>
#include "rpy_history.hpp"
#include "text_renderer.hpp"

struct Graph_Rect {
    float x, y, w, h;
};

/**
 * @brief Draws the rpy graph (background panel, gridlines, three coloured line series,
 *        legend, axis labels) inside 'rect". Note that the y-axis is a fixed [-180,180]
 *        deg range.
 * 
 * @param renderer The SDL_Renderer rendering context used to track all rendering
 *                  settings. Can only render within SDL_Window.
 * @param text The Text_Renderer object used to manage text rendering via SDL3_ttf
 * @param history The object storing the recent samples of roll/pitch/yaw readings and
 *                their timestamp
 * @param rect The struct used to store the properties of the outer panel that contains 
 *             the rpy_plot
 */
void draw_rpy_graph(SDL_Renderer *renderer, Text_Renderer &text, const Rpy_History &history, 
                    const Graph_Rect &rect);