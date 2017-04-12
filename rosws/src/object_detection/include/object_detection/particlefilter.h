#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H
#include <vector>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/mersenne_twister.hpp>
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
    static std::vector<int> importance_sampling( std::vector<double> weights, int number, double weight_max);

    static double random(double min, double max);


};

}

#endif // PARTICLEFILTER_H
