#include "object_detection/detection_pipeline.h"
namespace didi{

DetectionPipeline::DetectionPipeline() {
    // initialize sensor we use for detection
    boost::shared_ptr<Sensor> radar( new Radar());
    sensor_list_.push_back(radar);

}

void DetectionPipeline::mainLoop() {

    // 1. vehicle detection and propose new particles from the current measurement 1000
    for (size_t i = 0; i < sensor_list_.size(); i++) {
        std::vector <Vehicle_model> vehicles;
        sensor_list_[i] ->detect_vehicle(vehicles);
        // TODO: merge with the current detected vehicles
        // currently we only add vehicles
        if (detected_vehicle_list_.size() == 0 && vehicles.size() != 0 ) {
            for (size_t j = 0; j < vehicles.size(); j++ ) {
                detected_vehicle_list_.push_back(vehicles[j]);
            }
        }
    }
    // 2. create samples for each vehicle
    for (size_t i = 0 ; i < detected_vehicle_list_.size(); i++) {
        detected_vehicle_list_[i].create_particles(500);
    }

    // 3. compute weights for each particles
    for ( size_t i = 0 ; i < detected_vehicle_list_.size(); i++) {

        for (size_t j = 0 ; j <  detected_vehicle_list_[i].particles_.size(); j++ ) {
            double weight = 0;
            // TODO: combine weight for all the sensing modality w = 1/3 *w1 + 1/3 *w2 + 1/3 *w3
            for (size_t k = 0; k < sensor_list_.size(); k++) {
                weight =  sensor_list_[k]->compute_weight( detected_vehicle_list_[i].particles_[j]);
            }
            detected_vehicle_list_[i].particles_[j].weight_ = weight;
        }
    }
    // 4. resampling 500 particles


    // 5. compute the mean and variance


    // 6. perform vehicle dynamics on target vehicles and ego vehicle




}




}
