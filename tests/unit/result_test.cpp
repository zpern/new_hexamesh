#include <stdexcept>
#include <string>

#include <boundary_mesh/core/result.hpp>

int main()
{
    using boundary_mesh::Result;

    auto success = Result<int, std::string>::success(42);

    if (!success.hasValue())
    {
        return 1;
    }

    if (success.value() != 42)
    {
        return 2;
    }

    bool success_error_threw = false;

    try
    {
        (void)success.error();
    }
    catch (const std::logic_error &)
    {
        success_error_threw = true;
    }

    if (!success_error_threw)
    {
        return 3;
    }

    auto failure =
        Result<int, std::string>::failure("invalid input");

    if (failure.hasValue())
    {
        return 4;
    }

    if (failure.error() != "invalid input")
    {
        return 5;
    }

    bool failure_value_threw = false;

    try
    {
        (void)failure.value();
    }
    catch (const std::logic_error &)
    {
        failure_value_threw = true;
    }

    if (!failure_value_threw)
    {
        return 6;
    }

    const auto const_success =
        Result<int, std::string>::success(7);

    if (const_success.value() != 7)
    {
        return 7;
    }

    return 0;
}