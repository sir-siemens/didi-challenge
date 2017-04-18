#include "object_detection/particlefilter.h"

namespace didi {

boost::random::mt19937 ParticleFilter::rng_;

ParticleFilter::ParticleFilter() {

}

std::vector<int> ParticleFilter::importance_sampling( std::vector<double> normalized_weights, int number, double weight_max) {
    double  multi_factor = 1000;
    std::map<int, double> weights_map;
    int j = 0;
    double acc_weight_multiply = multi_factor * normalized_weights[j];

    for (int i = 0; i < multi_factor; i++ ) {

        if ( i <= acc_weight_multiply ) {
            weights_map[i] = j;
        }

        else {
            while ( i > acc_weight_multiply) {
                j++;
                acc_weight_multiply+= multi_factor * normalized_weights[j];
            }
            weights_map[i] = j;
        }
    }
    std::vector<int> particle_index (number);
    // dice uniform between 0 to multi_factor
    for (int i = 0 ; i < number; i++) {
        int rand_num = random_int( 0 , multi_factor );
        particle_index[i] = weights_map[rand_num];
    }
    return particle_index;
}

double ParticleFilter::random(double min, double max)  {
    boost::random::mt19937 rng;
    boost::random::uniform_real_distribution<> rand(min,max);
    return rand(rng);
}

int ParticleFilter::random_int(int min, int max) {
    boost::random::uniform_int_distribution<> rand(min,max);
    return rand(ParticleFilter::rng_);
}



}
