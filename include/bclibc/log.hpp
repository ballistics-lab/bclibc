#ifndef BCLIBC_LOG_HPP
#define BCLIBC_LOG_HPP

#include <cerrno>
#include <climits>
#include <string>
#include <cstdio>    // For vsnprintf
#include <cstdarg>   // For va_list
#include <cstdlib>   // For getenv
#include <algorithm> // For std::max

namespace bclibc
{

    // Logging macros are intentional: __FILE__, __LINE__, __func__ are only accessible
    // via preprocessor macros in C++11/14/17. When migrating to C++20, replace with
    // bclibc::log() using std::source_location.

    // Define Log Levels (matching Python's logging module for consistency)
    enum class BCLIBC_LogLevel
    {
        CRITICAL = 50,
        ERROR = 40,
        WARNING = 30, // Default for fprintf warnings
        INFO = 20,
        DEBUG = 10,
        NOTSET = 0
    };

    // --- ANSI Color Definitions ---
    // Defined as static const members for C++ usage
    static constexpr const char *BCLIBC_ANSI_COLOR_RED = "\x1b[31m";
    static constexpr const char *BCLIBC_ANSI_COLOR_YELLOW = "\x1b[33m";
    static constexpr const char *BCLIBC_ANSI_COLOR_CYAN = "\x1b[36m";
    static constexpr const char *BCLIBC_ANSI_BOLD_RED = "\x1b[1;31m";
    static constexpr const char *BCLIBC_ANSI_COLOR_BLUE = "\x1b[34m";
    static constexpr const char *BCLIBC_ANSI_COLOR_RESET = "\x1b[0m";
    static constexpr const char *BCLIBC_ANSI_BOLD_MAGENTA = "\x1b[1;35m";

    /**
     * @brief Determines the minimum configured log level.
     *
     * This function uses a static local variable initialized by a lambda to
     * safely check the environment variable `BCLIBC_LOG_LEVEL` exactly once
     * upon the first call, replacing the C-style global variable and init function.
     *
     * @return The currently configured minimum log level.
     */
    inline BCLIBC_LogLevel &get_min_level()
    {
        static BCLIBC_LogLevel level = []() -> BCLIBC_LogLevel
        {
            if (const char *env_level_str = std::getenv("BCLIBC_LOG_LEVEL"))
            {
                // strtol rather than std::stoi: no exceptions, so this header works where they are disabled.
                // Anything that is not a number in the range of an int keeps the default.
                char *end = nullptr;
                errno = 0;
                long level_val = std::strtol(env_level_str, &end, 10);
                if (end == env_level_str || errno == ERANGE || level_val > INT_MAX || level_val < INT_MIN)
                {
                    return BCLIBC_LogLevel::CRITICAL;
                }
                // Ensure the level is not negative
                return static_cast<BCLIBC_LogLevel>(std::max(0L, level_val));
            }
            return BCLIBC_LogLevel::CRITICAL; // Default: logging disabled
        }();
        return level;
    }

    /**
     * @brief Converts BCLIBC_LogLevel to its string representation.
     */
    inline const char *level_to_string(BCLIBC_LogLevel level)
    {
        switch (level)
        {
        case BCLIBC_LogLevel::CRITICAL:
            return "CRITICAL";
        case BCLIBC_LogLevel::ERROR:
            return "ERROR";
        case BCLIBC_LogLevel::WARNING:
            return "WARNING";
        case BCLIBC_LogLevel::INFO:
            return "INFO";
        case BCLIBC_LogLevel::DEBUG:
            return "DEBUG";
        default:
            return "NOTSET";
        }
    }

    /**
     * @brief Selects the ANSI color code based on BCLIBC_LogLevel.
     */
    inline const char *level_to_color(BCLIBC_LogLevel level)
    {
        switch (level)
        {
        case BCLIBC_LogLevel::CRITICAL:
            return BCLIBC_ANSI_BOLD_MAGENTA;
        case BCLIBC_LogLevel::ERROR:
            return BCLIBC_ANSI_COLOR_RED;
        case BCLIBC_LogLevel::WARNING:
            return BCLIBC_ANSI_COLOR_YELLOW;
        case BCLIBC_LogLevel::INFO:
            return BCLIBC_ANSI_COLOR_CYAN;
        case BCLIBC_LogLevel::DEBUG:
            return BCLIBC_ANSI_COLOR_BLUE;
        default:
            return BCLIBC_ANSI_COLOR_RESET;
        }
    }

    /**
     * @brief Core logging implementation using va_list for printf-style arguments.
     * @note This internal function allows the C++ function 'log' to support the
     * variadic arguments required by the original C-style macro API.
     */
    inline void log_impl_v(BCLIBC_LogLevel level, const char *file, int line, const char *func, const char *format, va_list args)
    {
#if defined(__wasm__) && !defined(BCLIBC_WASM_LOG)
        // A bare WebAssembly module has nowhere to write: printing would make it import WASI's fd_write, and a
        // module that imports nothing runs anywhere. Define BCLIBC_WASM_LOG to have the log back (it needs WASI).
        (void)level;
        (void)file;
        (void)line;
        (void)func;
        (void)format;
        (void)args;
#else
        // --- 1. Format the user message into a std::string ---

        // Copy va_list to safely determine size (required for portable vsnprintf use)
        va_list args_copy;
        va_copy(args_copy, args);
        // Determine required buffer size (not including null terminator)
        int size = std::vsnprintf(nullptr, 0, format, args_copy);
        va_end(args_copy);

        if (size < 0)
        {
            // Error in formatting
            std::fputs("Log formatting error.\n", stderr);
            return;
        }

        // Create buffer for the formatted message
        std::string message_buffer(size, 0);
        // Write the formatted message into the buffer
        std::vsnprintf(&message_buffer[0], size + 1, format, args);

        // --- 2. Construct and output the final log line, on the standard error stream ---

        std::fprintf(
            stderr,
            "%s%s%s: %s:%d in %s: %s\n",
            level_to_color(level),
            level_to_string(level),
            BCLIBC_ANSI_COLOR_RESET,
            file,
            line,
            func,
            message_buffer.c_str());
        std::fflush(stderr); // to make sure it is out at once
#endif
    }

    /**
     * @brief User-facing logging function.
     *
     * @param level The log level of the current message.
     * @param file The file name (__FILE__).
     * @param line The line number (__LINE__).
     * @param func The function name (__func__).
     * @param format The printf-style format string.
     * @param ... The arguments for the format string.
     */
    inline void log(BCLIBC_LogLevel level, const char *file, int line, const char *func, const char *format, ...)
    {
#if defined(__wasm__) && !defined(BCLIBC_WASM_LOG)
        // Nothing is printed here (see log_impl_v), and reading the level would pull in getenv, which makes a bare
        // module import WASI's environ_get and environ_sizes_get.
        (void)level;
        (void)file;
        (void)line;
        (void)func;
        (void)format;
        return;
#endif
        if (static_cast<int>(level) < static_cast<int>(get_min_level()))
        {
            return;
        }

        va_list args;
        va_start(args, format);
        log_impl_v(level, file, line, func, format, args);
        va_end(args);
    }

}; // namespace bclibc

#ifndef BCLIBC_ENABLE_DEBUG_LOGGING

#define BCLIBC_LOG(level, format, ...) \
    do                                 \
    {                                  \
    } while (0)
#define BCLIBC_NOTSET(format, ...) \
    do                             \
    {                              \
    } while (0)
#define BCLIBC_DEBUG(format, ...) \
    do                            \
    {                             \
    } while (0)

/*
#define BCLIBC_INFO(format, ...) \
    do                           \
    {                            \
    } while (0)
#define BCLIBC_WARN(format, ...) \
    do                           \
    {                            \
    } while (0)
#define BCLIBC_ERROR(format, ...) \
    do                            \
    {                             \
    } while (0)
#define BCLIBC_CRITICAL(format, ...) \
    do                               \
    {                                \
    } while (0)
*/

#else

// --- Convenience Macro for C-style API Compatibility ---
// The original BCLIBC_LOG macro is retained to automatically pass metadata
#define BCLIBC_LOG(level, format, ...) \
    bclibc::log(level, __FILE__, __LINE__, __func__, format, ##__VA_ARGS__)

#define BCLIBC_NOTSET(format, ...) \
    bclibc::log(bclibc::BCLIBC_LogLevel::NOTSET, __FILE__, __LINE__, __func__, format, ##__VA_ARGS__)

#define BCLIBC_DEBUG(format, ...) \
    bclibc::log(bclibc::BCLIBC_LogLevel::DEBUG, __FILE__, __LINE__, __func__, format, ##__VA_ARGS__)

#endif // BCLIBC_ENABLE_DEBUG_LOGGING

#define BCLIBC_INFO(format, ...) \
    bclibc::log(bclibc::BCLIBC_LogLevel::INFO, __FILE__, __LINE__, __func__, format, ##__VA_ARGS__)

#define BCLIBC_WARN(format, ...) \
    bclibc::log(bclibc::BCLIBC_LogLevel::WARNING, __FILE__, __LINE__, __func__, format, ##__VA_ARGS__)

#define BCLIBC_ERROR(format, ...) \
    bclibc::log(bclibc::BCLIBC_LogLevel::ERROR, __FILE__, __LINE__, __func__, format, ##__VA_ARGS__)

#define BCLIBC_CRITICAL(format, ...) \
    bclibc::log(bclibc::BCLIBC_LogLevel::CRITICAL, __FILE__, __LINE__, __func__, format, ##__VA_ARGS__)

#endif // BCLIBC_LOG_HPP
