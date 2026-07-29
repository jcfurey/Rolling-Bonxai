#pragma once

#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>

namespace RollingBonxai
{
    class Logger
    {
    public:
        using SharedPtr = std::shared_ptr<Logger>;

        // Default constructor
        Logger() = default;

        // Virtual destructor, need to prevent UB
        virtual ~Logger() = default;

        virtual void log_debug(const std::string& str) = 0;
        virtual void log_info(const std::string& str) = 0;
        virtual void log_warn(const std::string& str) = 0;
        virtual void log_error(const std::string& str) = 0;
        virtual void log_fatal(const std::string& str) = 0;
    };

    template<typename ... Args>
    std::string str_fmt( const std::string& format, Args ... args )
    {
        int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
        if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
        auto size = static_cast<size_t>( size_s );
        std::unique_ptr<char[]> buf( new char[ size ] );
        std::snprintf( buf.get(), size, format.c_str(), args ... );
        return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
    }
}
