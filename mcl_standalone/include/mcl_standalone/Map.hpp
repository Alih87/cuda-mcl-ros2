#pragma once

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>
#include <tuple>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <random>
#include <fstream>
#include <sstream>

namespace Map {

    constexpr std::size_t MAP_SIZE = 55;
    constexpr std::size_t MAP_RESOLUTION = 50;
    constexpr std::size_t OBSTACLE_DENSITY = 4;
    constexpr std::size_t STATE_DIM = 3;

    constexpr std::size_t AGENT = 128;
    constexpr std::size_t OBSTACLE = 0;
    constexpr std::size_t FREE = 255;
    constexpr std::size_t UNEXPLORED = 75;

    constexpr std::size_t N_RAYS = 1500;
    constexpr std::size_t RADIUS = 500;

    using newMap = Eigen::Matrix<int, MAP_SIZE, MAP_SIZE>;
    using State = Eigen::Vector<double, STATE_DIM>;
    using READING = Eigen::Vector<double, N_RAYS>;


    class CreateMap {

        public:
            CreateMap() {
            }

            void save_map(newMap& m_, std::string name_) {
                std::ofstream outFile(name_);

                if (!outFile) {
                    throw std::runtime_error("Error opening file.");
                }

                outFile << "P2\n";
                outFile << MAP_SIZE << " " << MAP_SIZE << "\n";
                outFile << "255\n";

                for (int y=0; y<m_.rows(); y++) {
                    for (int x=0; x<m_.cols(); x++) {
                        outFile << m_(y,x) << " ";
                    }
                    outFile << "\n";
                }

                outFile.close();
            }

            void load_map(newMap& m_, std::string name_) {
                std::ifstream readFile(name_);

                if (!readFile.is_open()) {
                    throw std::runtime_error("Error reading file");
                }

                newMap map_;
                std::string dummy;
                int px_val;

                for (int i=0; i<3; i++) {
                    std::getline(readFile, dummy);
                }

                for (int y=0; y<map_.rows(); y++) {
                    for (int x=0; x<map_.cols(); x++) {
                        if (readFile >> px_val) {
                            map_(y,x) = px_val;
                        } else {
                            throw std::runtime_error("Premature end of data.");
                        }
                    }
                }
                m_ = map_;
            }

            std::pair<int, int> locate(const newMap& m_, std::size_t px_) {
                for (int y=0; y<m_.rows(); y++) {
                    for (int x=0; x<m_.cols(); x++) {
                        if (m_(y,x) == px_) {
                            return {y,x};
                        }
                    }
                }

                return {-1,-1};
            }

            void update_map(newMap& m_, const std::pair<int, int>& loc, State deltaState) {
                int curr_y, curr_x;
                int next_y, next_x;
                std::tie(curr_y, curr_x) = loc;

                double curr_xd = static_cast<double>(curr_x);
                double curr_yd = static_cast<double>(curr_y);

                next_x = static_cast<int>(curr_xd + deltaState(0) / map_resolution);
                next_y = static_cast<int>(curr_yd + deltaState(1) / map_resolution);

                if (next_y < 0 || next_y >= m_.rows() || next_x < 0 || next_x >= m_.cols()) {
                    return;
                }

                m_(curr_y, curr_x) = FREE;
                m_(next_y, next_x) = AGENT;
            }

            State update_pos(const State& X, const State& X_) {
                State delta_X;
                double curr_y, curr_x, theta_;

                curr_y = X(0) - X_(0);
                curr_x = X(1) - X_(1);
                theta_ = X(2) - X_(2);

                double wrapped = std::fmod(theta_ + M_PI, 2.0 * M_PI);

                if (wrapped < 0) {
                    wrapped += 2.0 * M_PI;
                }

                theta_ = wrapped - M_PI;
                delta_X << curr_x, curr_y, theta_;

                return delta_X;
            }

            void spawn_obstacles_and_agent(newMap& m_) {
                std::random_device rd;
                std::mt19937 gen(rd());

                const std::vector<double> radii = {MAP_SIZE/32., MAP_SIZE/28, MAP_SIZE/25., MAP_SIZE/20., MAP_SIZE/16., MAP_SIZE/12.};

                std::uniform_int_distribution<int> distr_x(0, static_cast<int>(MAP_SIZE) - 1);
                std::uniform_int_distribution<int> distr_y(0, static_cast<int>(MAP_SIZE) - 1);
                std::uniform_int_distribution<int> distr_bin(0, 1);
                std::uniform_int_distribution<std::size_t> distr_radii(0, radii.size() - 1);

                for (std::size_t i=0; i<OBSTACLE_DENSITY; i++) {
                    int center_y = distr_y(gen);
                    int center_x = distr_x(gen);

                    double radius = radii[distr_radii(gen)] / map_resolution;

                    int idx_top_y = static_cast<int>(std::floor(center_y - radius));
                    int idx_down_y = static_cast<int>(std::ceil(center_y + radius));
                    int idx_left_x = static_cast<int>(std::floor(center_x - radius));
                    int idx_right_x = static_cast<int>(std::ceil(center_x + radius));

                    if (idx_top_y < 0) idx_top_y = 0;
                    if (idx_left_x < 0) idx_left_x = 0;
                    if (idx_down_y > static_cast<int>(MAP_SIZE) - 1) idx_down_y = static_cast<int>(MAP_SIZE) - 1;
                    if (idx_right_x > static_cast<int>(MAP_SIZE) - 1) idx_right_x = static_cast<int>(MAP_SIZE) - 1;

                    if (distr_bin(gen) == 0) {
                        for (int yy=idx_top_y; yy<=idx_down_y; yy++) {
                            for (int xx=idx_left_x; xx<=idx_right_x; xx++) {
                                m_(yy,xx) = OBSTACLE;
                            }
                        }
                    } else {
                        for (int yy=idx_top_y; yy<=idx_down_y; yy++) {
                            for (int xx=idx_left_x; xx<=idx_right_x; xx++) {
                                if (std::hypot(center_y - yy, center_x - xx) <= radius) {
                                    m_(yy,xx) = OBSTACLE;
                                }
                            }
                        }
                    }
                }

                std::vector<std::pair<int,int>> free_cells;
                for (int yy=0; yy<static_cast<int>(MAP_SIZE); yy++) {
                    for (int xx=0; xx<static_cast<int>(MAP_SIZE); xx++) {
                        if (m_(yy,xx) != OBSTACLE) {
                            free_cells.emplace_back(yy,xx);
                        }
                    }
                }

                if (free_cells.empty()) {
                    throw std::runtime_error("Cannot spawn agent: no free cells");
                }

                std::uniform_int_distribution<std::size_t> distr_free(0, free_cells.size() - 1);

                auto [agent_y, agent_x] = free_cells[distr_free(gen)];

                m_(agent_y,agent_x) = AGENT;

                std::cout << "Map populated with agent and obstacles\n";
            }

            newMap getCompositeMap(const std::vector<newMap>& maps_) {
                newMap compositeMap = maps_[0];
                for (const auto& m_ : maps_) {
                    compositeMap = (m_.array() == AGENT).select(AGENT, compositeMap);
                }
                return compositeMap;
            }

            void populate_(std::vector<newMap>& maps_, const newMap m_) {
                for (auto& map: maps_) {
                    map = m_;
                }
                std::cout << "Populated maps\n";
            }

            void initializeMap(newMap& m_) {
                m_.setConstant(UNEXPLORED);
            }
            
        private:
            static constexpr double map_resolution = static_cast<double>(MAP_RESOLUTION) / 100.0;
    };


    class LIDAR {
        public:
            newMap getScanMask(const newMap& m_, std::pair<int, int> loc) {
                int center_y, center_x;
                double curr_angle_rad = 0.0;
                double curr_radius_m = 0.0;
                newMap mask_;

                initializeMap(mask_, UNEXPLORED);
                std::tie(center_y, center_x) = loc;

                for (std::size_t i=0; i<N_RAYS; i++) {
                    while (curr_radius_m < radius_) {
                        int idx_y = center_y + static_cast<int>((curr_radius_m * std::cos(curr_angle_rad)) / map_resolution);
                        int idx_x = center_x + static_cast<int>((curr_radius_m * std::sin(curr_angle_rad)) / map_resolution);

                        if ((idx_y >= 0 && idx_x >= 0) && (idx_y < m_.rows() && idx_x < m_.cols())) {
                            if (idx_y == center_y && idx_x == center_x) {
                                mask_(idx_y,idx_x) = AGENT;
                            } else if (m_(idx_y,idx_x) == OBSTACLE) {
                                mask_(idx_y,idx_x) = OBSTACLE;
                                break;
                            } else {
                                mask_(idx_y,idx_x) = FREE;
                            }
                        } else {
                            break;
                        }
                        curr_radius_m += map_resolution;
                    }
                    curr_angle_rad += scan_resolution;
                    curr_radius_m = 0.0;
                }
                return mask_;
            }
            
            // The output is a vector of LIDAR readings starting from 0 degrees moving counter-clockwise.
            // Multiply the scan_resolution with the index to get the current angle.
            std::pair<READING, double> getReading(const newMap& m_, std::pair<int, int> loc) {
                double curr_angle_rad = 0.0;
                double curr_radius_m = 0.0;
                int center_y, center_x;
                READING reading_;
                reading_.setConstant(-1.);
                std::tie(center_y, center_x) = loc;

                if (m_(center_y, center_x) == OBSTACLE) {
                    std::cerr << "[WARN] Collision!\n";
                }

                for (std::size_t i=0; i<N_RAYS; i++) {
                    while (curr_radius_m < radius_) {
                        int idx_y = center_y + static_cast<int>((curr_radius_m * std::cos(curr_angle_rad)) / map_resolution);
                        int idx_x = center_x + static_cast<int>((curr_radius_m * std::sin(curr_angle_rad)) / map_resolution);

                        if ((idx_y >= 0 && idx_x >= 0) && (idx_y < m_.rows() && idx_x < m_.cols())) {
                            if (m_(idx_y,idx_x) == OBSTACLE) {
                                reading_(i) = curr_radius_m;
                                break;
                            }
                        } else {
                            break;
                        }
                        curr_radius_m += map_resolution;
                    }
                    curr_angle_rad += scan_resolution;                    
                    curr_radius_m = 0.0;
                }
                return {reading_, scan_resolution};
            }

            newMap map_from_reading(const std::pair<READING, double>& lidar_pair, const std::pair<int, int>& loc) {
                READING reading_;
                newMap lidar_pov;
                double scan_res;
                int center_y, center_x;

                std::tie(reading_, scan_res) = lidar_pair;
                initializeMap(lidar_pov, UNEXPLORED);
                std::tie(center_y, center_x) = loc;
                lidar_pov(center_y, center_x) = AGENT;

                for (int i=0; i<reading_.size(); i++) {
                    double angle = i * scan_res;
                    double ray_length = (reading_(i) < 0.0) ? radius_ : reading_(i);

                    for (double r=map_resolution; r<ray_length; r+=map_resolution) {
                        int idx_y = center_y + static_cast<int>((r * std::cos(angle)) / map_resolution);
                        int idx_x = center_x + static_cast<int>((r * std::sin(angle)) / map_resolution);

                        if (idx_y < 0 || idx_y >= lidar_pov.rows() || idx_x < 0 || idx_x >= lidar_pov.cols()) {
                            break;
                        }

                        lidar_pov(idx_y, idx_x) = FREE;
                    }

                    if (reading_(i) >= 0.0) {
                        int idx_y = center_y + static_cast<int>((reading_(i) * std::cos(angle)) / map_resolution);
                        int idx_x = center_x + static_cast<int>((reading_(i) * std::sin(angle)) / map_resolution);

                        if (idx_y >= 0 && idx_y < lidar_pov.rows() && idx_x >= 0 && idx_x < lidar_pov.cols()) {
                            lidar_pov(idx_y, idx_x) = OBSTACLE;
                        }
                    }
                }
                return lidar_pov;
            }

        private:
            void initializeMap(newMap& m_, const std::size_t state_) {
                m_.setConstant(state_);
            }

            static constexpr double map_resolution = static_cast<double>(MAP_RESOLUTION) / 100.0;
            static constexpr double radius_ = static_cast<double>(RADIUS) / 100.0;
            static constexpr double scan_resolution = (2.0 * M_PI) / static_cast<double>(N_RAYS);
    };

} // namespace Map
