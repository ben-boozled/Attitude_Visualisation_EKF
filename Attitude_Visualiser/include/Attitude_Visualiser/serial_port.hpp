/** 
 * @file serial_port.hpp
 * @brief A smol :3 wrapper around an OS serial handler. It does the following:
 *        1. Opens a port (e.g. COM3 for Windows, dev/ttyUSB0 for Linux).
 *        2. Reads newline-terminated text lines from the ESP32 (printed using printf).
 *        3. Performs the necessary resource cleanup thereafter.
 * 
 *        The platform split between Windows and Linux is kept entirely inside 
 *        serial_port.cpp to keep this header file and main.cpp platform-agnostic. The
 *        only platform-specific components in this header is the OS handle's storage
 *        type (see #ifdef preprocessor directives under "private:").
 * 
 *        The buffering behaviour (regardless of whether Windows or Linux is used):
 *        1. Keep an rx_buffer_ of the bytes received so far.
 *        2. To get a line, look for "\n" in the buffer. If there is one, return it
 *           without touching the OS.
 *        3. Else, it means there is a partial transmission. Read more bytes from the
 *           OS, append to the rx_buffer_, and repeat step 2.
 * 
 *        Sidenote: Many of the implementations and considerations here (e.g. destructor
 *                  calling close()) is similar to the way text_renderer.hpp has been implemented
 *                  since text_renderer also handles unique system resources.
 */

#pragma once

#include <string>

class Serial_Port {

public:
    Serial_Port() = default;    // tells the compiler to generate default constructor, even though other constructors are also declared here
    ~Serial_Port(); // the destructor calls close() to guarantee that the serial port device is released whenever the object goes out of scope

    /* Make the object non-copyable so that unique object instances owns unique OS resources */
    Serial_Port(const Serial_Port &) = delete;             // delete the copy constructor
    Serial_Port &operator=(const Serial_Port &) = delete;  // delete the copy assignment

    /**
     * @brief Opens the specified device. For example,
     *        Windows: e.g. "COM3" style names for USB-A ports
     *                 (also accepts "\\\\.\\COM10" form)
     *          
     *        POSIX: e.g. "dev/ttyUSB0" or "/dev/cu.usbserial-XXXX";
     *               Baud must match a standard POSIX baud value (from
     *               9600 to 230400)
     *        
     * @param path The path to the specified device name
     * @param baud The baud rate for communication
     * 
     * @return true if successful in opening, else returns false
     */
    bool open(const std::string &path, int baud);

    /**
     * @brief Closes the port if it is already opened. Safe to call even if
     *        port is already closed.
     */
    void close();

    /**
     * @brief Reports whether the port is opened.
     * 
     * @return true if port is opened, else returns false
     */
    bool is_open() const;

    /**
     * @brief A blocking read of one full line.
     * 
     * @param out_line The reference output parameter into which the line
     *                 is written
     * 
     * @return false on read error or if the port is closed, else returns
     *         true
     */
    bool read_line(std::string &out_line);

    /**
     * @brief Same as read_line() but returns after timeout_ms with false
     *        (i.e. a bounded read instead of a blocking read).
     * 
     * @param out_line The reference output parameter into which the line
     *                 is written, only written on success (i.e. a false
     *                 leaves it untouched)
     * @param timeout_ms The time limit before giving up
     * 
     * @return true if a full line is read within the timeout, else return
     *         false
     */
    bool read_line_timeout(std::string &out_line, int timeout_ms);

private:
#ifdef _WIN32
    void *handle_ = nullptr; // store a nullptr here instead of including <windows.h> to keep this header platform-agnostic; <window.h> is included in serial_port.cpp
#else // for Linux
    int fd_ = -1;    // file descriptor in POSIX, -1 == closed
#endif
    std::string rx_buffer_; // accumulates bytes received from the port
};