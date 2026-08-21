#include <chrono>
#include <cstddef>
#include <iostream>

#include <boundary_mesh/quality/volume_cell_evaluator.hpp>

int main()
{
    using namespace boundary_mesh;

    const PrismPoints prism{
        Point3{0.0, 0.0, 0.0},
        Point3{1.0, 0.0, 0.0},
        Point3{0.0, 1.0, 0.0},
        Point3{0.0, 0.0, 0.01},
        Point3{1.0, 0.0, 0.01},
        Point3{0.0, 1.0, 0.01}};

    const HexaPoints hexa{
        Point3{0.0, 0.0, 0.0},
        Point3{1.0, 0.0, 0.0},
        Point3{1.0, 1.0, 0.0},
        Point3{0.0, 1.0, 0.0},
        Point3{0.0, 0.0, 0.01},
        Point3{1.0, 0.0, 0.01},
        Point3{1.0, 1.0, 0.01},
        Point3{0.0, 1.0, 0.01}};

    constexpr std::size_t iterations = 1'000'000;
    volatile Scalar checksum = Scalar{0};

    const auto start = std::chrono::steady_clock::now();
    for (std::size_t index = 0; index < iterations; ++index)
    {
        const auto prism_result = evaluatePrism(prism);
        const auto hexa_result = evaluateHexa(hexa);
        if (!prism_result.hasValue() || !hexa_result.hasValue())
        {
            return 1;
        }

        checksum += prism_result.value().signed_volume;
        checksum += hexa_result.value().signed_volume;
    }
    const auto finish = std::chrono::steady_clock::now();

    const double seconds =
        std::chrono::duration<double>(finish - start).count();
    const double candidate_count =
        static_cast<double>(iterations) * 2.0;

    std::cout << "evaluated candidates: " << candidate_count << '\n'
              << "elapsed seconds: " << seconds << '\n'
              << "candidates/s: " << candidate_count / seconds << '\n'
              << "checksum: " << checksum << '\n';

    return 0;
}
