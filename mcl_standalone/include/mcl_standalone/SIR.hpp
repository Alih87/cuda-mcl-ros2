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

namespace SIR {
    
    template <
    std::size_t StateDim,
    std::size_t NumParticles,
    std::size_t HZ
    >
    class SIRParticleFilter {

        using Weights = Eigen::Vector<double, NumParticles>;
        using Particles = Eigen::Matrix<double, NumParticles, StateDim>;
        using State = Eigen::Vector<double, StateDim>;
        using Noise = Eigen::Matrix<double, StateDim, StateDim>;

        public:
            SIRParticleFilter() : gen(std::random_device{}()) {
                dt = 1.0 / static_cast<double>(HZ);
                weights.setConstant(1.0 / static_cast<double>(NumParticles));
                State_.setZero();
                prevState_.setZero();
                Measure_.setZero();
                control.setZero();
                Particles_.setZero();
                newParticles_.setZero();
                measure_.setZero();
                processNoiseCov.setZero();
                measurementNoiseCov.setIdentity();
            }

            Weights redistribute_Weights(Weights &w) {
                for (int i=0; i<w.size(); i++) {
                    w(i) = 1.0 / static_cast<double>(w.size());
                }
                return w;
            }

            void populate_measure(State Z) {
                for (int i=0; i<measure_.rows(); i++) {
                    measure_.row(i) = Z.transpose();
                }
            }

            State add_noise(State x, Noise noise_cov) {
                if (noise_cov.isZero()) {
                    return x;
                }

                std::normal_distribution<double> normal_dist(0.0, 1.0);
                Eigen::LLT<Noise> llt(noise_cov);

                if(llt.info() != Eigen::Success) {
                    throw std::runtime_error("Covariance matrix is not positive definite");
                }

                Noise L = llt.matrixL();
                State Z = State::NullaryExpr([&]() {
                    return normal_dist(gen);
                });

                return x + L*Z;
            }

            State get_residual_xytheta(State z, State x) {
                State res = z - x;
                double diff = std::fmod(z(2) - x(2) + M_PI, 2.0 * M_PI);

                if (diff < 0) {
                    diff += 2.0 * M_PI;
                }

                res(2) = diff - M_PI;

                return res;
            }

            Weights calculate_likelihood(Particles x, Noise m_cov) {
                Weights likelihood_;
                Eigen::LLT<Noise> llt(m_cov);

                if(llt.info() != Eigen::Success) {
                    throw std::runtime_error("Measurement covariance matrix is not positive definite");
                }

                double determinant = m_cov.determinant();

                if (determinant <= 0.0 || !std::isfinite(determinant)) {
                    throw std::runtime_error("Invalid measurement covariance determinant");
                }

                double normalization = 1.0 / (
                    std::pow(2.0 * M_PI, static_cast<double>(StateDim) / 2.0)
                    * std::sqrt(determinant)
                );

                for (int i=0; i<x.rows(); i++) {
                    State res = get_residual_xytheta(
                        measure_.row(i).transpose(),
                        x.row(i).transpose()
                    );

                    State solved = llt.solve(res);
                    double mahalanobis = res.dot(solved);

                    likelihood_(i) = normalization * std::exp(-0.5 * mahalanobis);
                }

                return likelihood_;
            }

            Weights reweight_and_normalize(Weights w_, Weights likelihood) {
                Weights w = likelihood.cwiseProduct(w_);
                double sum = w.sum();

                if (sum <= std::numeric_limits<double>::epsilon() || !std::isfinite(sum)) {
                    w.setConstant(1.0 / static_cast<double>(NumParticles));
                    return w;
                }

                w /= sum;
                return w;
            }

            State update_state(State x, State u) {
                double theta = x(2);
                double vx = u(0);
                double vy = u(1);
                double omega = u(2);

                double world_vx = vx * std::cos(theta) - vy * std::sin(theta);
                double world_vy = vx * std::sin(theta) + vy * std::cos(theta);

                x(0) += world_vx * dt;
                x(1) += world_vy * dt;
                x(2) += omega * dt;

                double wrapped = std::fmod(x(2) + M_PI, 2.0 * M_PI);

                if (wrapped < 0) {
                    wrapped += 2.0 * M_PI;
                }

                x(2) = wrapped - M_PI;

                return x;
            }

            State measure(State x, Noise noise_cov) {
                return add_noise(x, noise_cov);
            }

            Particles resample(Particles p, Weights w) {
                double N = static_cast<double>(NumParticles);
                Particles new_p;
                Weights C, positions;
                double rndm_uniform = 0.0;

                std::uniform_real_distribution<double> uni_dist(0.0, 1.0/N);
                Eigen::Vector<int, NumParticles> indices;

                std::partial_sum(w.begin(), w.end(), C.begin());
                C(C.size()-1) = 1.0;
                rndm_uniform = uni_dist(gen);

                positions = Weights::LinSpaced(NumParticles, 0.0, (N-1.0)/N);

                for (int i=0; i<positions.size(); i++) {
                    positions(i) += rndm_uniform;
                }

                indices = Eigen::Vector<int, NumParticles>::Zero();

                int i=0;
                for (int j=0; j<NumParticles; j++) {
                    while (i < static_cast<int>(NumParticles)-1 && positions(j) > C(i)) {
                        i += 1;
                    }
                    indices(j) = i;
                }

                for (int i=0; i<NumParticles; i++) {
                    new_p.row(i) = p.row(indices(i));
                }

                return new_p;
            }

            Particles getNewParticles() {
                Weights likelihood_ = calculate_likelihood(Particles_, measurementNoiseCov);
                weights = reweight_and_normalize(weights, likelihood_);
                newParticles_ = resample(Particles_, weights);
                weights = redistribute_Weights(weights);

                return newParticles_;
            }

            State State_, prevState_, Measure_, control;
            Particles Particles_, newParticles_;
            Noise processNoiseCov;
            Noise measurementNoiseCov;

        private:
            Particles measure_;
            Weights weights;
            std::mt19937 gen;
            double dt;
    };


} // namespace SIR