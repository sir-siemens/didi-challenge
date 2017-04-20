#include "object_detection/sensor.h"

namespace didi{

Sensor::Sensor()
{
}
Sensor::~Sensor() {

}
void Sensor::detect_vehicle(std::vector <Vehicle_model> &vehicle_list ){
    ROS_ERROR("virtual function should be implemented in the derived class");
}
double Sensor::compute_weight( Particle p) {
    ROS_ERROR("virtual function should be implemented in the derived class");
}

void Sensor::preprocessing() {
    ROS_ERROR("virtual function should be implemented in the derived class");
}


}
