/** 
 * @file text_renderer.cpp
 * @brief Implementation of text_renderer.hpp        
 **/

#include "Attitude_Visualiser/text_renderer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <vector>

namespace {

// Font sizes in SDL_ttf points (~ pixels)
constexpr float font_points[3] = { 18.0f, 24.0f, 30.0f };

constexpr const char *bundle_font_rel_path = "assets/fonts/DejaVuSansMono.ttf";

bool file_exists(const std::string &path)
{
    std::error_code ec;
    return std::filesystem::is_regular_file(path, ec) && !ec;
}

std::vector<std::string> candidate_font_paths()
{
    std::vector<std::string> c; // buffer to store candidate font paths

    if (const char *env = std::getenv("ATTITUDE_VISUALISER_FONT")) {
        if (*env) c.emplace_back(env);
    }

    // font path next to the executable
    if (const char *base = SDL_GetBasePath()) {
        c.emplace_back(std::string(base) + bundle_font_rel_path);
    }

    // font path relative to the current working directory
    c.emplace_back(bundle_font_rel_path); 

    // System fallbacks for typical font paths (only for Windows and Linux)
#if defined(_WIN32)
    c.emplace_back("C:\\Windows\\Fonts\\consola.ttf");
    c.emplace_back("C:\\Windows\\Fonts\\cour.ttf");
#else
    c.emplace_back("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf");
    c.emplace_back("/usr/share/fonts/TTF/DejaVuSansMono.ttf");
#endif
    return c;
}

}

Text_Renderer::~Text_Renderer() { shutdown(); }

bool Text_Renderer::init(SDL_Renderer *renderer) 
{
    /* Reset and initialise the library */
    shutdown();   // cleans up any previous state
    
    if (!TTF_Init()) {
        std::fprintf(stderr, "TTF_init failed: %s\n", SDL_GetError());
        return false;
    } // start SDL_ttf
    ttf_initialised_ = true;

    /* Open the first font candidate that exists AND loads at every font size */
    std::vector<std::string> tried;
    std::string chosen;
    for (const std::string &path : candidate_font_paths()) {
        tried.push_back(path);
        if(!file_exists(path)) continue; // skip if font file does not exist

        bool all_ok = true;
        for (int i = 0; i < num_sizes; i++) {
            fonts_[i] = TTF_OpenFont(path.c_str(), font_points[i]);
            if (!fonts_[i]) {
                std::fprintf(stderr, "TTF_OpenFont('%s', %.0f) failed: %s\n", 
                             path.c_str(), font_points[i], SDL_GetError());
                all_ok = false;
                break;
            } // try to open at every size in font_points, stops if any size fails
        }
        if (all_ok) {
            chosen = path;
            break; // if all font sizes load successfully, save the path and exit the loop
        }
        for (int i = 0; i < num_sizes; i++ ) {
            TTF_CloseFont(fonts_[i]);
            fonts_[i] = nullptr;
        }
    }

    // Only runs if there are no fonts found
    if (chosen.empty()) {
        std::fprintf(stderr, "No usable font found uwu. Looked in: \n");
        for (const std::string &p : tried) std::fprintf(stderr, " %s\n", p.c_str());
        std::fprintf(stderr, "Set ATTITUDE_VISUALISER_FONT to a .ttf path, or run from a "
                             "build with the assets folder next to the project executable.\n");
        shutdown();
        return false;
    }

    /* Initialises the text engine 
       Note: In SDL3_ttf, the text engine is what draws the text. This engine
       exists within the overall SDL3 renderer. */
    engine_ = TTF_CreateRendererTextEngine(renderer);
    if (!engine_) {
        std::fprintf(stderr, "TTF_CreateRendererTextEngine failed: %s\n", SDL_GetError());
        shutdown();
        return false;
    }

    /* Measure and store line heights for consistent line spacing, regardless of content */
    for (int i = 0; i < num_sizes; i++) {
        TTF_Text *probe = TTF_CreateText(engine_, fonts_[i], "Ag", 0); // "A" and "g" are chosen to cover the upper and lower vertical height extents
        int w = 0, h = 0;
        if (probe && TTF_GetTextSize(probe, &w, &h)) {
            line_height_[i] = static_cast<float>(h);
        } else {
            line_height_[i] = font_points[i] * 1.25f; // fallback value
        }
        if (probe) TTF_DestroyText(probe);
    }

    /* Check for monospaced font within this block scope */
    {
        text_extent narrow = measure("iiiiiiii", Text_Size::Medium); // "i"s serve as the narrowest character for the lower width limit
        text_extent wide = measure("MMMMMMMM", Text_Size::Medium); // "M"s serve as the widest character for the upper width limit
        // Check whether the widths differ by more than 1 pixel
        if (std::abs(narrow.width - wide.width) > 1.0f) {
            std::fprintf(stderr, "Warning: '%s' is not monospaced. HUD columns will "
                                 "not line up (but everything is still drawn).\n", chosen.c_str());
        }
    }

    /* Report success! :D */
    std::fprintf(stderr, "Font: %s\n", chosen.c_str()); // logs chosen font
    return true;
}

void Text_Renderer::shutdown()
{
    /* Step 1: Destroy the text engine */
    if (engine_) {
        TTF_DestroyRendererTextEngine(engine_);
        engine_ = nullptr;
    }

    /* Step 2: Close the fonts used by the engine */
    for (int i = 0; i < num_sizes; i++) {
        if (fonts_[i]) {
            TTF_CloseFont(fonts_[i]);
            fonts_[i] = nullptr;
        }
        line_height_[i] = 0.0f; // resets line height to avoid stale measurements
    }

    /* Step 3: Quit the TTF library itself */
    if (ttf_initialised_) {
        TTF_Quit();
        ttf_initialised_ = false;
    }
}

text_extent Text_Renderer::layout(const std::string &text, float x, float y, Text_Size size,
                                  const SDL_Color *colour_or_null)
{
    /* Check whether the text engine is initialised and text is present */
    if (!ready() || text.empty()) return { 0.0f, 0.0f };

    const int idx = static_cast<int> (size);
    const float pitch = line_height_[idx]; // vertical distance between consecutive lines

    float max_width = 0.0f; // widest line so far
    float cur_y = y;        // y-coordinate of next line to be drawn
    int line_count = 0;

    size_t pos = 0; // start of the current line
    for (;;) {
        const size_t nl = text.find('\n', pos);     // looks for the next new line from pos
        const std::string line = text.substr(pos, nl == std::string::npos ? std::string::npos : nl - pos); // extracts the current line

        /* Text creation for non-empty lines */
        if (!line.empty()) {
            TTF_Text *t = TTF_CreateText(engine_, fonts_[idx], line.c_str(), 0);
            if (t) {
                int w = 0, h = 0;
                if (TTF_GetTextSize(t, &w, &h)) {
                    max_width = std::max(max_width, static_cast<float> (w)); // updates max width
                }
                if (colour_or_null) {
                    TTF_SetTextColor(t, colour_or_null->r, colour_or_null->g,
                                     colour_or_null->b, colour_or_null->a);
                    TTF_DrawRendererText(t, x, cur_y);
                }
                TTF_DestroyText(t);
            }
        }

        // Advance the y-coordinate of the line to be drawn
        line_count++;
        cur_y += pitch;

        if (nl == std::string::npos) break;
        pos = nl + 1;
    }
    return { max_width, static_cast<float> (line_count) * pitch };
}

text_extent Text_Renderer::measure(const std::string &text, Text_Size size)
{
    return layout(text, 0.0f, 0.0f, size, nullptr);
}

text_extent Text_Renderer::draw(const std::string &text, float x, float y, 
                                Text_Size size, SDL_Color colour)
{
    return layout(text, x, y, size, &colour);
}