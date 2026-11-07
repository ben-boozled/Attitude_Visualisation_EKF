/** 
 * @file text_renderer.hpp
 * @brief Performs text rendering via SDL3_ttf. 
 *  
 * @note
 * Text_Renderer class owns the SDL3_ttf state, so in main.cpp it must be (1) initialised 
 * AFTER SDL_Renderer exists and (2) shutdown BEFORE the renderer is destroyed. 
 * 
 * If the font cannot be found/opened, init() returns false and the 3D view and graph lines
 * in SDL3 will continue to work, albeit without any labels      
 **/

#pragma once

#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>

#include <string>

struct text_extent {
    float width;
    float height;
};

/* Three fixed sizes to draw everything in the visualiser: graph labels,
   rotation HUD, and the key hints */
enum class Text_Size { Small = 0, Medium = 1, Large = 2 };

class Text_Renderer {

public:
    Text_Renderer() = default;  // default constructor

    /**
     * @brief The class destructor (runs at the end of the object's lifetime).
     *        Text_Renderer owns resources under SDL3_ttf (e.g. a text engine). 
     *        This destructor calls shutdown() to properly quit these additional 
     *        resources; a standard compiler-generated destructor would only destroy
     *        the pointers to these resources and not the resources themselves, which
     *        would cause resource leak.
     */
    ~Text_Renderer();           

    /* Ensure that unique object instances own unique resources */
    Text_Renderer(const Text_Renderer &) = delete;  // tells the compiler that the copy constructor is forbidden
    Text_Renderer &operator=(const Text_Renderer &) = delete; // tells the compiler that the copy assignment is forbidden

    /**
     * @brief Initialises SDL3_ttf, finds and opens the font at each size, and creates
     *        the text engine for "renderer". Font search order under candidate_font_paths():
     *        1. $ATTITUDE_VISUALISER_FONT$ (if set).
     *        2. assets/fonts/DejaVuSansMono.ttf next to the executable (copied over by
     *           the CMake post-build step).
     *        3. The same relative path to the fonts from the current working directory.
     *        4. Common system monospace font locations.
     *          
     *        A monospace font is expected (i.e. every character takes up the same width)
     *        because the HUD's columns are aligned with spaces.
     *  
     * @param renderer The SDL_Renderer rendering context used to track all rendering
     *                  settings. Can only render within SDL_Window.
     */
    bool init(SDL_Renderer *renderer);

    /**
     * @brief Resources are shutdown in reverse of how they were created by init. The
     *        shutdown sequence is:
     *        1. Destroy the text engine.
     *        2. Close the fonts.
     *        3. Quit the SDL3_ttf library.
     *        Note: The SDL3_ttf text engine must be destroyed BEFORE the overall SDL3
     *        renderer itself is destroyed.
     */
    void shutdown();

    /**
     * @brief Gets the state of the SDL3_ttf text engine.
     * 
     * @return Returns true if the text engine has been initialised. Else returns false.
     */
    bool ready() const { return engine_ != nullptr; }

    /**
     * @brief Getter function for the line height
     * 
     * @return The line height in pixels
     */
    float line_height(Text_Size size) const { return line_height_[static_cast<int>(size)]; }

    /**
     * @brief Measures the maximum width and height of the text to be drawn.
     * 
     * @param text The text to be measured
     * @param size The enum corresponding to the required text size
     * 
     * @return Struct containing the maximum text width and the height (number of lines 
     *         x pitch).
     */
    text_extent measure(const std::string &text, Text_Size size);

    /**
     * @brief Draws the text in the specified colour.
     * 
     * @param text The text to be drawn
     * @param x, y The coordinates to draw the text
     * @param size The enum corresponding to the required text size
     * @param colour The rgba values of the text
     * 
     * @return Struct containing the maximum text width and the height (number of lines 
     *         x pitch).
     */    
    text_extent draw(const std::string &text, float x, float y, Text_Size size, SDL_Color colour);

private:
    static constexpr int num_sizes = 3;

    /**
     * @brief Helper function for measure() and draw(). Lays out (and optionally draws)
     *        a block of multi-line text. using the same function for measure() and
     *        draw() ensures the measured and drawn sizes always agree
     * 
     * @param text The text to be measured/drawn
     * @param x, y The coordinates to draw the text (enter 0 for measure)
     * @param size The enum corresponding to the required text size
     * @param colour_or_null If this is null, draws nothing. Else, draws the text
     *                       in the specified colour.
     * 
     * @return Struct containing the maximum text width and the height (number of lines 
     *         x pitch).
     */
    text_extent layout(const std::string &text, float x, float y, Text_Size size,
                       const SDL_Color *colour_or_null);

    TTF_Font *fonts_[num_sizes] = { nullptr, nullptr, nullptr };
    float line_height_[num_sizes] = { 0.0f, 0.0f, 0.0f };   // in pixels, determined during initialisation
    TTF_TextEngine *engine_ = nullptr;
    bool ttf_initialised_ = false;  // records whether TTF_Quit() is required in shutdown()
};