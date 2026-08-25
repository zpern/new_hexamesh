#pragma once

namespace spdlog
{
    template <typename... Args>
    void info(const char *, Args &&...)
    {
    }

    template <typename... Args>
    void warn(const char *, Args &&...)
    {
    }
}
