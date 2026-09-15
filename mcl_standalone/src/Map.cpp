#include <iostream>
#include <cstdlib>
#include <vector>
#include <string>
#include <memory>
#include <utility>
#include <tuple>
#include <filesystem>
#include <sstream>
#include <iomanip>
#include <mcl_standalone/Map.hpp>
#include <mcl_standalone/SIR.hpp>

using namespace Map;

void show_map(const Map::newMap& map_) {
    for (int y=0; y<map_.rows(); y++) {
        for (int x=0; x<map_.cols(); x++) {
            std::cout << map_(y,x) << " ";
        }
        std::cout << std::endl;
    }
}

std::string make_filename(const std::string& dir_, const std::string& prefix_, int idx_) {
    std::stringstream ss_;
    ss_ << dir_ << "/" << prefix_ << "_" << std::setw(5) << std::setfill('0') << idx_ << ".pgm";
    return ss_.str();
}

void load_env_once();

int main() {
    load_env_once();

    const char* env_mcl_dir = std::getenv("MCL_STANDALONE_DIR");
    std::string MCL_DIR = env_mcl_dir;

    std::unique_ptr<std::string> map_path_ = std::make_unique<std::string>(MCL_DIR + R"(/maps/1.pgm)");
    std::unique_ptr<std::string> map_dir_spawned_ = std::make_unique<std::string>(MCL_DIR + R"(/maps/spawned)");
    std::unique_ptr<std::string> map_dir_composite_ = std::make_unique<std::string>(MCL_DIR + R"(/maps/composite)");
    std::unique_ptr<std::string> map_dir_lidar_ = std::make_unique<std::string>(MCL_DIR + R"(/maps/lidar)");

    std::filesystem::remove_all(*map_dir_spawned_);
    std::filesystem::remove_all(*map_dir_composite_);
    std::filesystem::remove_all(*map_dir_lidar_);
    std::filesystem::create_directories(*map_dir_spawned_);
    std::filesystem::create_directories(*map_dir_lidar_);
    std::filesystem::create_directories(*map_dir_composite_);

    auto sir_obj = SIR::SIRParticleFilter<Map::STATE_DIM, Map::MAP_SIZE, Map::MAP_SIZE>();
    auto lidar_obj = Map::LIDAR();
    auto map_obj = Map::CreateMap();

    sir_obj.measurementNoiseCov = Eigen::Matrix3d::Identity() * 2e-2;
    sir_obj.processNoiseCov = Eigen::Matrix3d::Identity() * 3e-1;
    sir_obj.control << 7.8, 2.8, 0.16;

    Map::newMap map_;
    map_obj.initializeMap(map_);
    map_obj.spawn_obstacles_and_agent(map_);
    //map_obj.load_map(map_, *map_path_);
    
    auto [agent_y, agent_x] = map_obj.locate(map_, Map::AGENT);
    if (agent_y == -1 || agent_x == -1) {
        throw std::runtime_error("Agent not found!");
    }

    Map::newMap initial_map_ = map_;
    Map::State initial_state_ = sir_obj.prevState_;

    int iteration_ = 0;

    while (true) {
        sir_obj.State_ = sir_obj.update_state(sir_obj.prevState_, sir_obj.control);
        sir_obj.Measure_ = sir_obj.measure(sir_obj.State_, sir_obj.measurementNoiseCov);

        std::vector<Map::newMap> maps(sir_obj.Particles_.rows());

        for (int i=0; i<sir_obj.Particles_.rows(); ++i) {
            Map::State prev_particle = sir_obj.Particles_.row(i);
            Map::State predicted = sir_obj.update_state(prev_particle, sir_obj.control);
            sir_obj.Particles_.row(i) = sir_obj.add_noise(predicted, sir_obj.processNoiseCov);
            Map::newMap aux_map = initial_map_;
            Map::State particle_pos = map_obj.update_pos(sir_obj.Particles_.row(i), initial_state_);
            map_obj.update_map(aux_map, map_obj.locate(aux_map, Map::AGENT), particle_pos);
            maps[i] = std::move(aux_map);
        }

        Map::newMap composite_map = map_obj.getCompositeMap(maps);
        map_ = initial_map_;
        Map::State agent_pos = map_obj.update_pos(sir_obj.State_, initial_state_);
        map_obj.update_map(map_, map_obj.locate(map_, Map::AGENT), agent_pos);
        sir_obj.populate_measure(sir_obj.Measure_);

        auto reading_pair = lidar_obj.getReading(map_, map_obj.locate(map_, Map::AGENT)); 
        Map::newMap lidar_scan_map = lidar_obj.map_from_reading(reading_pair, map_obj.locate(map_, Map::AGENT));

        sir_obj.newParticles_ = sir_obj.getNewParticles();
        sir_obj.Particles_ = sir_obj.newParticles_;

        std::string spawned_path_ = make_filename(*map_dir_spawned_, "spawned", iteration_);
        std::string composite_path_ = make_filename(*map_dir_composite_, "composite", iteration_);
        std::string lidar_path_ = make_filename(*map_dir_lidar_, "lidar", iteration_);

        map_obj.save_map(map_, spawned_path_);
        map_obj.save_map(composite_map, composite_path_);
        map_obj.save_map(lidar_scan_map, lidar_path_);

        sir_obj.prevState_ = sir_obj.State_;
        iteration_++;
    }

    return 0;
}
