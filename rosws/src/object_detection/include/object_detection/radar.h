#ifndef RADAR_H
#define RADAR_H
#include <object_detection/sensor.h>
namespace didi{

class Radar : public Sensor
{
public:
    Radar();

    virtual void detect_vehicle(std::vector <Vehicle_model> &vehicle_list);

    virtual double compute_weight( Particle p) ;

};

}
#endif // RADAR_H
