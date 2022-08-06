#include "main.hpp"
#include <vector>
#include <iostream>
#include <cassert>

int main()
{
    std::vector<Boid> boids;
    int n_boids; //number of boids, to enter in input
    if(n_boids<=0) throw E_InvalidNumberOfBoids{};
    for(int i=1; i<=n_boids; ++i) {
        boids[i].updateVelocity();
        boids[i].moveBoid();
    }
   
}