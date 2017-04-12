#ifndef SENSOR_H
#define SENSOR_H
#include <object_detection/vehicle_model.h>

namespace didi{


class Sensor
{
    /**
      abstract class sensor
      */
public:
    Sensor();

    virtual void detect_vehicle(std::vector <Vehicle_model> &vehicle_list)  = 0;

    virtual double compute_weight( Particle p);


};
}

#endif // SENSOR_H
