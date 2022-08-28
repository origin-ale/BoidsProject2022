#include "stats.hpp"

std::vector<double> getDistances(std::vector<Boid> const& boids_1, std::vector<Boid> const& boids_2){
  std::vector<double> dists;
  for(int j = 0; j < static_cast<int>(boids_1.size()); ++j){
      for(int k = 0; k < static_cast<int>(boids_2.size()); ++k){
        dists.push_back(sqrt((boids_1[j].getPosition() - boids_2[k].getPosition()).getNorm2()));
      }
  }
  return dists;
}

std::vector<double> getSpeeds(std::vector<Boid> const& boids){
  std::vector<double> speeds;
  for(int i = 0; i < static_cast<int>(boids.size()); ++i){
    speeds.push_back(sqrt(boids[i].getVelocity().getNorm2()));
  }
  return speeds;
}

Stats getStats(std::vector<double> const& vector){
    Stats result;

    double sum = std::accumulate(vector.begin(), vector.end(), 0.);
    result.mean = sum / vector.size();

    std::vector<double> vector_diff(vector.size());
    std::transform(vector.begin(), vector.end(), vector_diff.begin(), [result](double x) { return x - result.mean; });
    double vector_sq_sum = std::inner_product(vector_diff.begin(), vector_diff.end(), vector_diff.begin(), 0.0);
    result.stdev = std::sqrt(vector_sq_sum / vector.size());

    return result;
}

void makeHisto(std::vector<double> const& data, const char* histo_title, const char* stat_title, double max_bin, TCanvas& target_canvas, int target_pad){

  TH1D histo = TH1D(stat_title, histo_title, 100, 0., max_bin);
  for(double x: data) histo.Fill(x);
  target_canvas.cd(target_pad);

  gStyle->SetOptStat(1100);

  histo.DrawCopy("hist", "");
  target_canvas.Modified();
  target_canvas.Update();
}