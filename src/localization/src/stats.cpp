#include <random>
#include <iostream>
#include <algorithm>

#include "stats.hpp"

using namespace std;

default_random_engine random_engine_stats;

vector<int> multinomial_sample(int n_trials, vector<float> ps) {
    vector<int> res (ps.size());
    discrete_distribution<int> discrete_sampler (ps.begin(), ps.end());
    for (int i = 0; i < n_trials; i++) {
        res[discrete_sampler(random_engine_stats)]++;
    }
    return res;
}
