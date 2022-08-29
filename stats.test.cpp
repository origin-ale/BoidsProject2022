#include "stats.hpp"
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <algorithm>
#include <cmath>
#include <iostream>

#include "doctest.h"

TEST_CASE("Testing getFlockDistances") {
  std::vector<Boid> flockdist_test;
  flockdist_test.push_back(
      Boid{Position(0., 0.), Velocity(0., 0.), Angle(0.), 0});
  flockdist_test.push_back(
      Boid{Position(1., 0.), Velocity(1., 1.), Angle(0.), 0});
  flockdist_test.push_back(
      Boid{Position(2., 2.), Velocity(2., 3.), Angle(10.), 0});

  flockdist_test.push_back(
      Boid{Position(0., 1.), Velocity(0., 0.), Angle(0.), 1});
  flockdist_test.push_back(
      Boid{Position(0., 2.), Velocity(0., 0.), Angle(0.), 1});

  std::vector<double> flock0_dists = getFlockDistances(flockdist_test, 0);
  std::vector<double> flock1_dists = getFlockDistances(flockdist_test, 1);

  std::vector<double> flock0_comp{1., std::sqrt(8), std::sqrt(5)};
  std::vector<double> flock1_comp{1.};

  CHECK(flock0_dists == flock0_comp);
  CHECK(flock1_dists == flock1_comp);
}

TEST_CASE("Testing getPredFlockDistances") {
  std::vector<Boid> predflockdist_test;
  predflockdist_test.push_back(
      Boid{Position(0., 0.), Velocity(0., 0.), Angle(0.), 0});
  predflockdist_test.push_back(
      Boid{Position(1., 0.), Velocity(1., 1.), Angle(0.), 0});
  predflockdist_test.push_back(
      Boid{Position(2., 2.), Velocity(2., 3.), Angle(10.), 0});

  predflockdist_test.push_back(
      Boid{Position(0., 1.), Velocity(0., 0.), Angle(0.), 1});
  predflockdist_test.push_back(
      Boid{Position(0., 2.), Velocity(0., 0.), Angle(0.), 1});

  std::vector<Boid> pred_test;
  pred_test.push_back(Boid{Position(3., 3.), Velocity(0., 0.), Angle(0.), 0});
  pred_test.push_back(Boid{Position(4., 3.), Velocity(10., 0.), Angle(4.), 1});

  std::vector<double> predflock0_dists =
      getPredFlockDistances(predflockdist_test, 0, pred_test);
  std::vector<double> predflock1_dists =
      getPredFlockDistances(predflockdist_test, 1, pred_test);

  std::vector<double> predflock0_comp{std::sqrt(18.), std::sqrt(13.),
                                      std::sqrt(2.),  5.,
                                      std::sqrt(18.), std::sqrt(5.)};
  std::vector<double> predflock1_comp{std::sqrt(13.), std::sqrt(10.),
                                      std::sqrt(20.), std::sqrt(17.)};

  CHECK(predflock0_dists == predflock0_comp);
  CHECK(predflock1_dists == predflock1_comp);
}

TEST_CASE("Testing getFlockSpeeds") {
  std::vector<Boid> flockspeed_test;
  flockspeed_test.push_back(
      Boid{Position(0., 0.), Velocity(0., 0.), Angle(0.), 0});
  flockspeed_test.push_back(
      Boid{Position(1., 0.), Velocity(1., 1.), Angle(0.), 0});
  flockspeed_test.push_back(
      Boid{Position(2., 2.), Velocity(0., 3.), Angle(10.), 0});

  flockspeed_test.push_back(
      Boid{Position(0., 1.), Velocity(10., 0.), Angle(0.), 1});
  flockspeed_test.push_back(
      Boid{Position(0., 2.), Velocity(5., 4.), Angle(0.), 1});

  std::vector<double> flock0_speeds = getFlockSpeeds(flockspeed_test, 0);
  std::vector<double> flock1_speeds = getFlockSpeeds(flockspeed_test, 1);

  std::vector<double> flock0_comp{0., std::sqrt(2.), 3.};
  std::vector<double> flock1_comp{10., std::sqrt(41.)};

  CHECK(flock0_speeds == flock0_comp);
  CHECK(flock1_speeds == flock1_comp);
}

TEST_CASE("Testing fillDrawHisto") {}