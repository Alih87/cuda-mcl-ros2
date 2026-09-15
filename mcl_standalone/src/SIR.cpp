#include <iostream>
#include <mcl_standalone/SIR.hpp>

using namespace SIR;

int main() {
    auto sir_obj = SIRParticleFilter<3, 100, 100>();

    sir_obj.measurementNoiseCov = Eigen::Matrix3d::Identity() * 1e-3;
    sir_obj.processNoiseCov = Eigen::Matrix3d::Identity() * 1e-3;
    sir_obj.control << 0.1, 0.0, 0.09;
    
    while (true) {
        sir_obj.State_ << sir_obj.update_state(sir_obj.prevState_, sir_obj.control);
        sir_obj.Measure_ << sir_obj.measure(sir_obj.State_, sir_obj.measurementNoiseCov);

        for (int i = 0; i < sir_obj.Particles_.rows(); i++) {
            auto predicted = sir_obj.update_state(sir_obj.Particles_.row(i), sir_obj.control);
            sir_obj.Particles_.row(i) = sir_obj.add_noise(predicted, sir_obj.processNoiseCov);
        }
        
        sir_obj.populate_measure(sir_obj.Measure_);
        sir_obj.newParticles_ = sir_obj.getNewParticles();
        sir_obj.Particles_ = sir_obj.newParticles_;
        sir_obj.prevState_ << sir_obj.State_;

        std::cout << "True State: "
                << sir_obj.State_.transpose()
                << std::endl;
        std::cout << "Measured State: "
                << sir_obj.Measure_.transpose()
                << std::endl;
        std::cout << "Particles 1: "
                << sir_obj.newParticles_.row(0)
                << std::endl;
        std::cout << "Particles 2: "
                << sir_obj.newParticles_.row(1)
                << std::endl;
        std::cout << std::endl;
    }

    return 0;
}