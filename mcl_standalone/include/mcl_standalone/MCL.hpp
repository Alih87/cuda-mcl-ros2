#pragma once

#include <array>
#include <cstdint>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>
#include <limits>
#include <eigen3/Eigen/Dense>
#include <random>
#include <numeric>
#include <SIR.hpp>

using namespace SIR;

namespace MCL {

    template <std::size_t StateDim,
              std::size_t NumParticles,
              std::size_t HZ,
              std::size_t MapSize, SIRParticleFilter<StateDim, NumParticles, HZ>>
        class MCLEstimator {

            using Map = Eigen::Matrix<double, MapSize, MapSize>;

            public:
                MCLEstimator() {
                }

            private:
                Map map;
    };

} // namespace MCL