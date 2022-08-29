#include <vector>
#include <algorithm>
#include <numeric>
#include "boids.hpp"
#include <TH1.h>
#include <TCanvas.h>
#include <TStyle.h>
#include <TPaveStats.h>
#include <TLatex.h>

std::vector<double> getFlockDistances(std::vector<Boid> const&, int);
std::vector<double> getPredFlockDistances(std::vector<Boid> const&, int, std::vector<Boid> const&);
std::vector<double> getFlockSpeeds(std::vector<Boid> const&, int);

void fillDrawHisto(TH1D &, std::vector<double> const&, Color_t const&, float);