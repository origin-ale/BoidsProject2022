#include "stats.hpp"

std::vector<double> getFlockDistances(std::vector<Boid> const& boids, int flock){
  std::vector<double> dists;
  for(int j = 0; j < static_cast<int>(boids.size()); ++j){
      for(int k = j+1; k < static_cast<int>(boids.size()); ++k){
        if(boids[j].getFlock() == flock && boids[j].getFlock() == boids[k].getFlock()) 
          dists.push_back(sqrt((boids[j].getPosition() - boids[k].getPosition()).getNorm2()));
      }
  }
  return dists;
}

std::vector<double> getPredFlockDistances(std::vector<Boid> const& boids, int flock, std::vector<Boid> const& preds){
  std::vector<double> dists;
  for(int j = 0; j < static_cast<int>(preds.size()); ++j){
      for(int k = 0; k < static_cast<int>(boids.size()); ++k){
        if(boids[k].getFlock() == flock) dists.push_back(sqrt((preds[j].getPosition() - boids[k].getPosition()).getNorm2()));
      }
  }
  return dists;
}

std::vector<double> getFlockSpeeds(std::vector<Boid> const& boids, int flock){
  std::vector<double> speeds;
  for(int i = 0; i < static_cast<int>(boids.size()); ++i){
    if(boids[i].getFlock()==flock)
      speeds.push_back(sqrt(boids[i].getVelocity().getNorm2()));
  }
  return speeds;
}

void fillDrawHisto(TH1D & histo, std::vector<double> const& data, Color_t const& histo_color, float statbox_offset){
  for(double x: data) histo.Fill(x);
  histo.UseCurrentStyle();
  histo.SetLineColor(histo_color);
  histo.DrawCopy("SAME", "");
  gPad->Update();

  double x_center = gPad->GetUxmax();
  double x_width = 0.4 * gPad->GetUxmax();
  double y_center = (gPad->GetUymax()) * (1 - (statbox_offset * 0.3));
  double y_width = 0.1 * gPad->GetUymax();

  TPaveStats statbox = TPaveStats(x_center - x_width, y_center + y_width, x_center + y_width, y_center - y_width, "NB");
  statbox.SetBorderSize(1);
  statbox.SetLineColor(histo_color);
  statbox.SetFillColor(0);
  gPad->Modified();
  gPad->Update();

  std::string stat_name_histo = histo.GetName();
  std::string stat_name_base = "_statbox_offset_";
  std::string stat_name = stat_name_histo + stat_name_base + std::to_string(statbox_offset);
  TList *listOfLines = statbox.GetListOfLines();

  TText *histo_name_txt = new TLatex(0,0, histo.GetName());
  histo_name_txt->SetTextFont(42);
  histo_name_txt->SetTextSize(0.04);
  histo_name_txt->SetTextColor(kBlack);

  std::string histo_mean_label = "Mean: ";
  std::string histo_mean = std::to_string(histo.GetMean());
  TText *histo_mean_txt = new TLatex(0,0, (histo_mean_label+histo_mean).c_str());
  histo_mean_txt->SetTextFont(42);
  histo_mean_txt->SetTextSize(0.04);
  histo_mean_txt->SetTextColor(kBlack);

  std::string histo_stdev_label = "Std Dev: ";
  std::string histo_stdev = std::to_string(histo.GetStdDev());
  TText *histo_stdev_txt = new TLatex(0,0, (histo_stdev_label+histo_stdev).c_str());
  histo_stdev_txt->SetTextFont(42);
  histo_stdev_txt->SetTextSize(0.04);
  histo_stdev_txt->SetTextColor(kBlack);

  listOfLines->Add(histo_name_txt);
  listOfLines->Add(histo_mean_txt);
  listOfLines->Add(histo_stdev_txt);

  gPad->Modified();
  gPad->Update();
  statbox.DrawClone();
  gPad->Modified();
  gPad->Update();

}