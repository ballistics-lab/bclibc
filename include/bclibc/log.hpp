#ifndef BCLIBC_LOG_HPP
#define BCLIBC_LOG_HPP

#ifndef BCLIBC_BUILD_NATMOD
#include <iostream>
#include <string>
#include <cstdarg>   // For va_list
#include <cstdlib>   // For getenv
#include <algorithm> // For std::max
#endif
#include <cstdio>    // For vsnprintf (used by the natmod no-op path)

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

#ifdef BCLIBC_BUILD_NATMOD
    /* Natmod builds: all logging is disabled.  No iostream, no string, no .data
     * sections.  The static const is constant-initialised → goes to .rodata. */
    inline BCLIBC_LogLevel &get_min_level() noexcept {
        static const BCLIBC_LogLevel s = BCLIBC_LogLevel::CRITICAL;
        return const_cast<BCLIBC_LogLevel&>(s);
    }
    inline void log(BCLIBC_LogLevel, const char*, int, const char*,
                    const char*, ...) noexcept {}
#else
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
                try
                {
                    int level_val = std::stoi(env_level_str);
                    return static_cast<BCLIBC_LogLevel>(std::max(0, level_val));
                }
                catch (const std::exception &e)
                {
                    return BCLIBC_LogLevel::CRITICAL;
                }
            }
            return BCLIBC_LogLevel::CRITICAL;
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

    inline void log_impl_v(BCLIBC_LogLevel level, const char *file, int line, const char *func, const char *format, va_list args)
    {
        va_list args_copy;
        va_copy(args_copy, args);
        int size = std::vsnprintf(nullptr, 0, format, args_copy);
        va_end(args_copy);

        if (size < 0)
        {
            std::cerr << "Log formatting error.\n";
            return;
        }

        std::string message_buffer(size, 0);
        std::vsnprintf(&message_buffer[0], size + 1, format, args);

        const char *color = level_to_color(level);
        const char *level_str = level_to_string(level);

        std::cerr << color
                  << level_str
                  << BCLIBC_ANSI_COLOR_RESET << ": "
                  << file << ":" << line
                  << " in " << func << ": "
                  << message_buffer
                  << "\n";

        std::cerr.flush();
    }

    inline void log(BCLIBC_LogLevel level, const char *file, int line, const char *func, const char *format, ...)
    {
        if (static_cast<int>(level) < static_cast<int>(get_min_level()))
        {
            return;
        }

        va_list args;
        va_start(args, format);
        log_impl_v(level, file, line, func, format, args);
        va_end(args);
    }

#endif // BCLIBC_BUILD_NATMOD

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
