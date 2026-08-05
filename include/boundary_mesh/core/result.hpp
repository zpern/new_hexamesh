#pragma once

#include <cstddef>
#include <stdexcept>
#include <utility>
#include <variant>

namespace boundary_mesh
{
    template <class T, class E>
    class Result
    {
    public:
        static Result success(T value)
        {
            return Result{
                std::in_place_index<0>,
                std::move(value)};
        }

        static Result failure(E error)
        {
            return Result{
                std::in_place_index<1>,
                std::move(error)};
        }

        bool hasValue() const noexcept
        {
            return storage_.index() == 0;
        }

        T &value()
        {
            if (!hasValue())
            {
                throw std::logic_error(
                    "Result does not contain a value");
            }

            return std::get<0>(storage_);
        }

        const T &value() const
        {
            if (!hasValue())
            {
                throw std::logic_error(
                    "Result does not contain a value");
            }

            return std::get<0>(storage_);
        }

        E &error()
        {
            if (hasValue())
            {
                throw std::logic_error(
                    "Result does not contain an error");
            }

            return std::get<1>(storage_);
        }

        const E &error() const
        {
            if (hasValue())
            {
                throw std::logic_error(
                    "Result does not contain an error");
            }

            return std::get<1>(storage_);
        }

    private:
        template <std::size_t Index, class U>
        Result(
            std::in_place_index_t<Index> index,
            U &&value)
            : storage_(
                  index,
                  std::forward<U>(value))
        {
        }

        std::variant<T, E> storage_;
    };
}
