#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H
#include <vector>
#include <map>
#include <boost/random/uniform_real.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <ros/ros.h>
namespace didi {

class ParticleFilter
{
public:
    ParticleFilter();

    /**
      weights: A list containing weights
      weight_max: the maximal weight in the weights list
      number: number of particles to be drawn
      return: the index of particles
      */
    static std::vector<int> importance_sampling( std::vector<double> normalized_weights, int number, double weight_max);

    static double random(double min, double max);

    static int random_int(int min, int max);

    static boost::random::mt19937 rng_;

};

}

#endif // PARTICLEFILTER_H
