#include "object_detection/particlefilter.h"

namespace didi {

ParticleFilter::ParticleFilter() {

}

std::vector<int> ParticleFilter::importance_sampling( std::vector<double> weights, int number, double weight_max) {
    std::vector<int> particle_index (number);
    double beta = 0;
    int index = 0;
    for (int i = 0; i < number;i++) {
        beta = beta + random(0, 2*weight_max);
        while ( weights[index] < beta ) {
            beta = beta - weights[index];
            index = index + 1;
            if (index = weights.size()){
                index = 0;
            }
        }
        //
        particle_index[i] = index;
    }
    return particle_index;
}

double ParticleFilter::random(double min, double max)  {
    boost::random::mt19937 rng;
    boost::random::uniform_real_distribution<> rand(min,max);
    return rand(rng);
}


}
