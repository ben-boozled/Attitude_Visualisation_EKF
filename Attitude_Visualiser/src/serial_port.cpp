/** 
 * @file serial_port.cpp
 * @brief Implementation of serial_port.hpp. For Windows, CreateFile/ReadFile/SetCommState
 *        are used. POSIX termios is used everywhere else.    
 **/

#include "Attitude_Visualiser/serial_port.hpp"

#ifdef _WIN32

/** ==================================================== 
 *  Windows Implementation
 *  ==================================================== */
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX        // for <windows.h>, else it defines min/max which breaks <algorithm> =(
#include <windows.h>

namespace {

/**
 * @brief Helper function to prepend the \\.\ prefix into the port name, as 
 *        required by Windows to access device namespace.
 * 
 * @param path The port name
 * 
 * @return The port name in the required format by Windows
 */
std::string normalise_port_path(const std::string &path)
{
    if (path.rfind("\\\\.\\", 0) == 0) return path; // path already has the \\.\ prefix 
    if (path.size() >= 3 && 
        (path[0] == 'C' || path[0] == 'c') && 
        (path[1] == 'O' || path[1] == 'o') &&
        (path[2] == 'M' || path[2] == 'm')) {
        return "\\\\.\\" + path;
    } // prepend the required prefix (the bare COMX fails if X is >= 10)
    return path;
}

/**
 * @brief Configures how long Readfile may wait on this handle, such that Readfile 
 *        will return as soon as at least one byte is available, or after timeout_ms.
 * 
 * @param h The Windows handle to the open serial port
 * @param timeout_ms How long a read may wait for data (in ms)
 * 
 * @return false only on a hard SetCommTmeouts failure (e.g. bad handle),
 *         else returns true
 */
bool arm_read_timeout(HANDLE h, DWORD timeout_ms)
{
    COMMTIMEOUTS timeouts{};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = timeout_ms;
    return SetCommTimeouts(h, &timeouts) != 0;
}

/**
 * @brief Performs a single ReadFile call under the timeout armed above.
 *        Also clears any pending communication error state first (because
 *        several common USB chips e.g. CP210x, CH340, FTDI) can occasionally
 *        leave the port stuck and refuse to read further data, causing a
 *        perpetual 0 bytes read issue.     
 * 
 * @param h  The Windows handle to the open serial port
 * @param buf The destination buffer where the bytes read from the serial
 *            port get written
 * @param buf_size The number of bytes to read
 * 
 * @return The number of bytes read (0 on a genuine timeout with no data),
 *         or -1 on a hard error.
 */
long read_chunk(HANDLE h, char *buf, DWORD buf_size)
{
    DWORD errors = 0;
    ClearCommError(h, &errors, nullptr);    // clears any latched error state e.g. framing, overrun

    DWORD n = 0;    // stores the number of bytes read
    if (!ReadFile(h, buf, buf_size, &n, nullptr)) {
        return -1;
    } // the actual read process
    return static_cast<long>(n);
}

}

// Calling close() guarantees that the serial port device is released whenever the 
// object goes out of scope, even if the caller forgets to explicityly close it
Serial_Port::~Serial_Port() { close(); }

bool Serial_Port::open(const std::string &path, int baud)
{
    close();

    HANDLE h = CreateFileA(normalise_port_path(path).c_str(),
                           GENERIC_READ | GENERIC_WRITE,
                           0,               // no sharing   
                           nullptr,         // default security
                           OPEN_EXISTING,
                           0,               // non-overlapped (i.e. blocking I/O)
                           nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        return false;
    } // check if port opening failed

    DCB dcb{};
    dcb.DCBlength = sizeof(dcb); // must set first before calling GetCommState API
    if(!GetCommState(h, &dcb)) {
        CloseHandle(h);
        return false;
    } // DCB = device control block (Win32 struct that holds config settings for serial port)

    // DCB settings
    dcb.BaudRate = static_cast<DWORD>(baud);    // note: DCB.Baudrate accepts any baudrate directly
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE; // disabled DTR and RTS because both are used by the ESP32 autoreset circuit
    dcb.fDsrSensitivity = FALSE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;

    if (!SetCommState(h, &dcb)) {
        CloseHandle(h);
        return false;
    } // applies the above dcb settings 

    if (!arm_read_timeout(h, 24u * 60u * 60u * 1000u)) {
        CloseHandle(h);
        return false;
    } // ~24 hour timeout (i.e. read_line() blocks forever)

    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR); // discards stale bytes in the rx and tx queues

    handle_ = h;
    rx_buffer_.clear();
    return true;
}

void Serial_Port::close()
{
    if (handle_) {
        CloseHandle(static_cast<HANDLE>(handle_));
        handle_ = nullptr;
    }
    rx_buffer_.clear(); // avoid data leak into next session    
}

bool Serial_Port::is_open() const { return handle_ != nullptr; }

bool Serial_Port::read_line(std::string &out_line)
{
    if (!handle_) return false;
    HANDLE h = static_cast<HANDLE>(handle_);

    for (;;) {
        auto nl = rx_buffer_.find('\n'); // look for new line
        if (nl != std::string::npos) {
            out_line = rx_buffer_.substr(0, nl); // copy the substring up till before the new line
            if (!out_line.empty() && out_line.back() == '/r') out_line.pop_back(); // remove the trailing '/r'
            rx_buffer_.erase(0, nl + 1);
            return true;
        }

        // if there is no new line in rx_buffer_, continue reading chunks to form a full line
        // append the new chunks to the rx_buffer_
        char chunk[256];
        long n = read_chunk(h, chunk, sizeof(chunk));
        if (n < 0) return false;
        if (n == 0) return false;
        rx_buffer_.append(chunk, static_cast<size_t>(n));

        // Clear buffer if size >4096bytes to protect against a sender that never sends a '\n' 
        if (rx_buffer_.size() > 4096) {
            rx_buffer_.clear();
        }
    }
}

bool Serial_Port::read_line_timeout(std::string &out_line, int timeout_ms)
{
    if (!handle_) return false;
    HANDLE h = static_cast<HANDLE>(handle_);

    auto nl = rx_buffer_.find('\n'); // an already buffered line from a previous call
    if (nl != std::string::npos) {
        out_line = rx_buffer_.substr(0, nl); // copy the substring up till before the new line
        if (!out_line.empty() && out_line.back() == '/r') out_line.pop_back(); // remove the trailing '/r'
        rx_buffer_.erase(0, nl + 1);
        return true;
    }

    // Rearm the driver with timeout_ms
    if (!arm_read_timeout(h, static_cast<DWORD>(timeout_ms))) {
        return false;
    }

    // if there is no new line in rx_buffer_, read one additional chunk
    // append the new chunk to the rx_buffer_
    char chunk[256];
    long n = read_chunk(h, chunk, sizeof(chunk));
    if (n <= 0) return false; // timed out (n == 0) or hard error (n < 0)
    
    rx_buffer_.append(chunk, static_cast<size_t>(n));
    // Clear buffer if size >4096bytes to protect against a sender that never sends a '\n' 
    if (rx_buffer_.size() > 4096) {
        rx_buffer_.clear();
    }

    // Search for a newline again
    nl = rx_buffer_.find('\n');
    if (nl == std::string::npos) {
        return false; // got bytes, but not a full line yet
    }

    // New full line is available; extract it
    out_line = rx_buffer_.substr(0, nl);
    if (!out_line.empty() && out_line.back() == '/r') out_line.pop_back();
    rx_buffer_.erase(0, nl + 1);
    return true;
}

#else // Linux
/** ==================================================== 
 *  Linux Implementation
 *  ==================================================== */
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/time.h>
#include <cerrno>
#include <cstring>

namespace {

/**
 * @brief Maps integers to termios constants
 * 
 * @param baud  Baud rate
 * @param out Output reference parameter for termios constant
 * 
 * @return false for non-standard (unsupported) baud rates, else returns true
 */
bool baud_to_constant(int baud, speed_t &out)
{
    switch (baud) {
        case 9600:     out = B9600;    return true;
        case 19200:    out = B19200;   return true;
        case 38400:    out = B38400;   return true;
        case 57600:    out = B57600;   return true;
        case 115200:   out = B115200;  return true;
        case 230400:   out = B230400;  return true;
        default:       return false;
    }
}

}

// Calling close() guarantees that the serial port device is released whenever the 
// object goes out of scope, even if the caller forgets to explicityly close it
Serial_Port::~Serial_Port() { close(); }

bool Serial_Port::open(const std::string &path, int baud)
{
    close();

    /* Check the baud rate */
    speed_t speed;
    if (!baud_to_constant(baud, speed)) {
        return false;
    }

    /* Select the global C open(), open the file device (i.e. port) */
    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY); // fd = file descriptor (-1 means not open)
    if (fd_ < 0) {
        return false;
    }
    fcntl(fd_, F_SETFL, 0); // switch to blocking mode (so that read() waits until data arrives)

    /* Load current terminal settings into tty */
    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        close();
        return false;
    }

    cfsetispeed(&tty, speed);   // input speed
    cfsetospeed(&tty, speed);   // output speed

    /* Configure the frame format and control lines */
    // 8N1 (8 data bits, no parity bits, 1 stop bit)
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;    // no hardware flow control (ESP32 PCB uses them for auto-reset circuit)
    tty.c_cflag |= CREAD | CLOCAL;

    /* Disable terminal behaviour */
    tty.c_lflag &= ~ICANON; // no line-by-line buffering, deliver bytes as they come
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ISIG;

    /* Disable input translation (to read the bytes exactly as the device sent )*/
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);   
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST; // disable output processing (if any)

    /* Set read behaviour */
    tty.c_cc[VMIN] = 1; // wait until at least 1 byte
    tty.c_cc[VTIME] = 0; // no timer

    /* Apply the modified settings */
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        close();
        return false;
    }

    tcflush(fd_, TCIOFLUSH); // clean stale bytes
    rx_buffer_.clear();
    return true;
}

void Serial_Port::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    rx_buffer_.clear();
}

bool Serial_Port::is_open() const { return fd_ >= 0; }

bool Serial_Port::read_line(std::string &out_line)
{
    if (fd_ < 0) return false;

    for (;;) {
        auto nl = rx_buffer_.find('\n'); // look for new line
        if (nl != std::string::npos) {
            out_line = rx_buffer_.substr(0, nl); // copy the substring up till before the new line
            if (!out_line.empty() && out_line.back() == '/r') out_line.pop_back(); // remove the trailing '/r'
            rx_buffer_.erase(0, nl + 1);
            return true;
        }

        // if there is no new line in rx_buffer_, continue reading chunks to form a full line
        // append the new chunks to the rx_buffer_
        char chunk[256];
        ssize_t n = ::read(fd_, chunk, sizeof(chunk));
        if (n < 0) {
            if (errno == EINTR) continue; // a signal interrupted the call but nothing is really wrong
            return false;
        }
        if (n == 0) {
            return false;
        }
        rx_buffer_.append(chunk, static_cast<size_t>(n));

        // Clear buffer if size >4096bytes to protect against a sender that never sends a '\n' 
        if (rx_buffer_.size() > 4096) {
            rx_buffer_.clear();
        }
    }
}

bool Serial_Port::read_line_timeout(std::string &out_line, int timeout_ms)
{
    if (fd_ < 0) return false;

    auto nl = rx_buffer_.find('\n'); // an already buffered line from a previous call
    if (nl != std::string::npos) {
        out_line = rx_buffer_.substr(0, nl); // copy the substring up till before the new line
        if (!out_line.empty() && out_line.back() == '/r') out_line.pop_back(); // remove the trailing '/r'
        rx_buffer_.erase(0, nl + 1);
        return true;
    }

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd_, &read_fds);

    timeval tv{};
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = ::select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
    if (ret < 0) {
        if (errno == EINTR) return false;   // treat as a timeout for this call
        return false;
    }
    if (ret == 0) {
        return false; // timed out, no date =(
    }

    char chunk[256];
    ssize_t n = ::read(fd_, chunk, sizeof(chunk));

    if (n <= 0) return false; // closed / error
    
    rx_buffer_.append(chunk, static_cast<size_t>(n));
    // Clear buffer if size >4096bytes to protect against a sender that never sends a '\n' 
    if (rx_buffer_.size() > 4096) {
        rx_buffer_.clear();
    }

    // Search for a newline again
    nl = rx_buffer_.find('\n');
    if (nl == std::string::npos) {
        return false; // got bytes, but not a full line yet
    }

    // New full line is available; extract it
    out_line = rx_buffer_.substr(0, nl);
    if (!out_line.empty() && out_line.back() == '/r') out_line.pop_back();
    rx_buffer_.erase(0, nl + 1);
    return true;
}

#endif // for #ifdef _WIN32