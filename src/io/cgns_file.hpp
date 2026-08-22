#pragma once

#include <utility>

#include <cgnslib.h>

namespace boundary_mesh
{
    // 只管理成功打开的 cgnslib 文件编号，确保所有返回路径都会关闭文件。
    class CgnsFile
    {
    public:
        explicit CgnsFile(int number) noexcept
            : number_(number)
        {
        }

        CgnsFile(const CgnsFile &) = delete;
        CgnsFile &operator=(const CgnsFile &) = delete;

        CgnsFile(CgnsFile &&other) noexcept
            : number_(std::exchange(other.number_, -1))
        {
        }

        CgnsFile &operator=(CgnsFile &&) = delete;

        ~CgnsFile()
        {
            if (number_ >= 0)
            {
                cg_close(number_);
            }
        }

        int number() const noexcept
        {
            return number_;
        }

    private:
        int number_{-1}; // cgnslib 文件编号，负值表示无所有权
    };
}
