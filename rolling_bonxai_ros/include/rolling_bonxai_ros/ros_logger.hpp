#pragma once

#include "rolling_bonxai/logger.hpp"
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <utility>

namespace RollingBonxai
{
    /**
     * @brief Logger implementation that forwards to an rclcpp::Logger
     *
     * @note Every message is passed through a literal "%s" format string.
     *       Forwarding a runtime string directly as the format argument is a
     *       format-string bug: any chunk path or policy name containing '%'
     *       would read bogus varargs. GCC 13 (Jazzy) also rejects it under
     *       -Wformat-security, which -Werror builds treat as an error.
     */
    class RosLogger : public Logger
    {
    public:
        using SharedPtr = std::shared_ptr<RosLogger>;

        explicit RosLogger(rclcpp::Logger logger) : logger_{std::move(logger)} {}
        ~RosLogger() override = default;

        void log_debug(const std::string& str) override
        {
            RCLCPP_DEBUG(logger_, "%s", str.c_str());
        }

        void log_info(const std::string& str) override
        {
            RCLCPP_INFO(logger_, "%s", str.c_str());
        }

        void log_warn(const std::string& str) override
        {
            RCLCPP_WARN(logger_, "%s", str.c_str());
        }

        void log_error(const std::string& str) override
        {
            RCLCPP_ERROR(logger_, "%s", str.c_str());
        }

        void log_fatal(const std::string& str) override
        {
            RCLCPP_FATAL(logger_, "%s", str.c_str());
        }

    private:
        rclcpp::Logger logger_;
    };
}
