#include <vector>
#include <algorithm>
#include <numeric>
#include "boids.hpp"
#include <TH1.h>
#include <TCanvas.h>
#include <TStyle.h>


struct Stats{
  double mean;
  double stdev;
};

std::vector<double> getDistances(std::vector<Boid> const&, std::vector<Boid> const&);
std::vector<double> getSpeeds(std::vector<Boid> const&);

Stats getStats(std::vector<double> const&);

void makeHisto(std::vector<double> const&, const char*, const char*, double, TCanvas&, int);