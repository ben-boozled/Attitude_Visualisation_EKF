#include <iostream>

#define SDL_MAIN_HANDLED    // tells SDL to not replace main with its own entry point (i.e. use the normal int main(int argc, char **argv))
#include <SDL3/SDL.h>
#include <thread>

#include "Attitude_Visualiser/orientation_math.hpp"
#include "Attitude_Visualiser/shared_orientation.hpp"
#include "Attitude_Visualiser/graph_renderer.hpp"
#include "Attitude_Visualiser/text_renderer.hpp"
#include "Attitude_Visualiser/rpy_history.hpp"
#include "Attitude_Visualiser/serial_port.hpp"
#include "Attitude_Visualiser/utils.hpp"

using namespace visualiser;

/** ========================================================
 *                        Main Program 
 * ======================================================= **/
int main(int argc, char **argv){
    /** --------------------------------------------------------
     *               Command Line Arguments 
    -------------------------------------------------------- **/
    std::string port;
    int baud = 115200;
    bool demo_mode = true; // only if there are no arguments

    // 1st argument == serial port name
    if (argc >= 2) {
        port = argv[1];
        demo_mode = false;
    }

    // 2nd argument == baud rate
    if (argc >= 3) {
        baud = std::atoi(argv[2]); // text-to-int conversion
    }

    /** --------------------------------------------------------
     *              Initialise SDL video subsystem 
    -------------------------------------------------------- **/
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window *window = SDL_CreateWindow("IMU Attitude Visualiser (ISM330DLC + EKF)", 
                                          window_w, window_h, 0);
    if (!window) {
        std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_Renderer *renderer = SDL_CreateRenderer(window, nullptr);
    if (!renderer) {
        std::fprintf(stderr, "SDL_CreateRenderer failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    /** --------------------------------------------------------
     *                      Text Setup 
    -------------------------------------------------------- **/
    Text_Renderer text;
    if (!text.init(renderer)) {
        std::fprintf(stderr, "Continuing without on-screen text.\n");
    }

    /** --------------------------------------------------------
     *           Shared State and Data Thread Setup 
    -------------------------------------------------------- **/
    Shared_Orientation shared;       // mutex protected
    std::atomic<bool> running{true}; // atomic shutdown flag

    std::thread data_thread;

    // Only one background thread starts (the render loop does not need to know which one)
    if (demo_mode) {
        std::fprintf(stderr, "No serial port given - running in demo mode "
                             "(synthetic rotation). Pass a port to use real hardware:\n"
                             "  %s COM3 115200            (Windows)\n"
                             "  %s /dev/ttyUSB0 115200    (Linux)\n", argv[0], argv[0]);
        data_thread = std::thread(demo_orientation_thread, &shared, &running);
    } else {
        data_thread = std::thread(serial_reader_thread, port, baud, &shared, &running);
    }

    /** --------------------------------------------------------
     *      Camera, History, and Per-session State Setup 
    -------------------------------------------------------- **/
    Projector proj;
    proj.screen_w = viewport_w;   // the 3D scene's own viewport, not the full size
    proj.screen_h = window_h;

    Rpy_History rpy_history(graph_window_sec);
    const auto start_time = std::chrono::steady_clock::now();

    // tracks the current window size
    float window_w_cur = static_cast<float>(window_w);  // current window width
    float window_h_cur = static_cast<float>(window_h);  // current window height

    double yaw_zero_offset = 0.0;   // for heading correction
    bool zero_yaw_requested = false;

    bool quit = false;

    /** --------------------------------------------------------
     *                 Main render loop
    -------------------------------------------------------- **/
    while (!quit) {
        /* Checks for user input to quit (non-blocking, runs once per frame) */
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT) quit = true;
            if (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_ESCAPE) quit = true;
            if (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_Z) zero_yaw_requested = true;
            if (event.type == SDL_EVENT_WINDOW_RESIZED) {
                window_w_cur = static_cast<float>(event.window.data1);
                window_h_cur = static_cast<float>(event.window.data2);

                proj.screen_w = std::max(1.0, static_cast<double>(window_w_cur) - graph_panel_w);
                proj.screen_h = static_cast<double>(window_h_cur);
            }
        }

        /* Get orientation and apply yaw zeroing */
        double age_sec = 0.0;
        Quat q = shared.get(age_sec);

        if (zero_yaw_requested) {
            double r_now, p_now, y_now;
            euler_from_quat(q, r_now, p_now, y_now);
            yaw_zero_offset = -y_now;
            zero_yaw_requested = false;
        }

        q = add_yaw(q, yaw_zero_offset);

        /* Display-only quaternion (so that +X is screen right)*/
        Quat q_visual = add_yaw(q, pi);

        /* Compute data staleness */
        double staleness01 = 0.0;
        if (age_sec > stale_after_sec) {
            staleness01 = std::min(1.0, (age_sec - stale_after_sec) / 
                                        (disconnected_after_sec - stale_after_sec));
        }

        /** --------------------------------------------------------
         *     Historise Euler Angles (in deg) for RPY Graph  
        -------------------------------------------------------- **/
        double roll, pitch, yaw;
        euler_from_quat(q, roll, pitch, yaw);

        double t_now = std::chrono::duration<double>
                       (std::chrono::steady_clock::now() - start_time).count();

        // Pushes a sample on every frame, even if a new q has not been received
        rpy_history.push(t_now, roll * 180.0 / pi, pitch * 180 / pi, yaw * 180 / pi);

        /** --------------------------------------------------------
         *                     Render visuals  
        -------------------------------------------------------- **/
        SDL_SetRenderDrawColor(renderer, 18, 18, 22, 255);
        SDL_RenderClear(renderer);

        draw_floor_grid(renderer, proj); // fixed-grid world reference
        draw_board_and_axes(renderer, proj, q_visual, staleness01);
        draw_rotation_hud(renderer, text, q, age_sec, 16.0f, 16.0f);

        // Hints for keyboard control
        text.draw("Z: Zero yaw    Esc: Quit", 16.0f,
                  window_h - text.line_height(Text_Size::Large) - 14.0f, Text_Size::Large,
                  SDL_Color{190, 195, 205, 255});

        // Side panel and RPY graph
        Graph_Rect graph_rect{
            window_w_cur - static_cast<float>(graph_panel_w), 0.0f,
            static_cast<float>(graph_panel_w), window_h,
        };
        draw_rpy_graph(renderer, text, rpy_history, graph_rect);

        /* Update the screen with all renderings performed since the previous call */
        SDL_RenderPresent(renderer);

        /** --------------------------------------------------------
         *              Frame pacing (target: ~60fps)  
        -------------------------------------------------------- **/
        SDL_Delay(16); // ms, render loop is independent of data rate from thread
    }

    /** --------------------------------------------------------
     *             Shutdown and Cleanup Functions 
    -------------------------------------------------------- **/
    /* Shutdown handshake between main() and background data thread */
    running.store(false); // tells data thread to stop
    if (data_thread.joinable()) {
        data_thread.join(); // pause here until the thread has finished (~200ms for actual, ~10ms for demo)
    } // check whether there is a thread to wait for

    /* Destroy SDL_ttf and SDL engines in reverse order of creation */
    text.shutdown();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
