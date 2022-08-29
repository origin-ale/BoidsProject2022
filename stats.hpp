#include <TCanvas.h>
#include <TH1.h>
#include <TLatex.h>
#include <TPaveStats.h>
#include <TStyle.h>

#include <algorithm>
#include <numeric>
#include <vector>

#include "boids.hpp"

std::vector<double> getFlockDistances(std::vector<Boid> const &, int);
std::vector<double> getPredFlockDistances(std::vector<Boid> const &, int,
                                          std::vector<Boid> const &);
std::vector<double> getFlockSpeeds(std::vector<Boid> const &, int);

void fillDrawHisto(TH1D &, std::vector<double> const &, Color_t const &, float);