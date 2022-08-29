#include "main.hpp"

int main() {
  time_t begin_time;
  time(&begin_time);
  tm* bgn_tm = localtime(&begin_time);

  bool debug = false;  // debug flag, skips parameter input

  int n_flocks;
  std::vector<int> flock_pops;
  int n_boids;
  double sep_factor;
  double align_factor;
  double cohes_factor;
  int n_preds;
  double sight_angle_degrees;

  if (debug) {
    n_flocks = 3;
    flock_pops = {34, 33, 33};
    sep_factor = 0.5;
    align_factor = 0.1;
    cohes_factor = 0.3;
    n_preds = 2;
    sight_angle_degrees = 60.;
  }

  else {
    // input parameters
    std::cout << "Enter number of flocks: ";
    std::cin >> n_flocks;
    while (n_flocks < 0 || !(std::isfinite(n_flocks))) {
      std::cout << "Invalid number of flocks entered. Please enter again: ";
      std::cin >> n_flocks;
    }

    for (int i = 0; i < n_flocks; ++i) {
      std::cout << "Enter number of boids in flock " << i + 1 << ": ";
      int input;
      std::cin >> input;
      flock_pops.push_back(input);
      while (flock_pops[i] < 0 || !(std::isfinite(flock_pops[i]))) {
        std::cout << "Invalid number of boids entered. Please enter again: ";
        std::cin >> flock_pops[i];
      }
    }

    std::cout << "Enter separation factor: ";
    std::cin >> sep_factor;  // recommended values around 0.5
    while (sep_factor < 0 || !(std::isfinite(sep_factor))) {
      std::cout << "Invalid separation factor entered. Please enter again: ";
      std::cin >> sep_factor;
    }

    std::cout << "Enter alignment factor: ";
    std::cin >> align_factor;  // recommended values around 0.1
    while (align_factor < 0 || align_factor >= 1 ||
           !(std::isfinite(align_factor))) {
      std::cout << "Invalid alignment factor entered. Please enter again: ";
      std::cin >> align_factor;
    }

    std::cout << "Enter cohesion factor: ";
    std::cin >> cohes_factor;  // recommended values around 0.3
    while (cohes_factor < 0 || !(std::isfinite(cohes_factor))) {
      std::cout << "Invalid cohesion factor entered. Please enter again: ";
      std::cin >> cohes_factor;
    }

    std::cout << "Enter number of predators: ";
    std::cin >> n_preds;
    while (n_preds < 0 || !(std::isfinite(n_preds))) {
      std::cout << "Invalid number of predators entered. Please enter again: ";
      std::cin >> n_preds;
    }

    std::cout << "Enter sight angle: ";
    std::cin >> sight_angle_degrees;
    while (sight_angle_degrees < 0 || sight_angle_degrees > 180 ||
           !(std::isfinite(sight_angle_degrees))) {
      std::cout << "Invalid sight angle entered. Please enter again: ";
      std::cin >> sight_angle_degrees;
    }
  }

  n_boids = std::accumulate(flock_pops.begin(), flock_pops.end(), 0);
  Angle sight_angle = Angle(sight_angle_degrees);

  // parameter assertions
  assert(n_boids >= 0 && std::isfinite(n_boids));
  assert(sep_factor >= 0 && std::isfinite(sep_factor));
  assert(align_factor >= 0 && align_factor < 1 && std::isfinite(align_factor));
  assert(cohes_factor >= 0 && std::isfinite(cohes_factor));
  assert(n_preds >= 0 && std::isfinite(n_preds));
  assert(sight_angle_degrees <= 180 && std::isfinite(sight_angle_degrees));

  // fixed parameter initialization
  int window_x = 1500;
  int window_y = 1500;
  double sim_radius = std::sqrt(MAX_RADIUS2);
  double scale =
      (std::max(std::min(window_x - 50., window_y - 50.), 1.) / sim_radius) /
      2;  // coord-to-pixel scaling factor, such that the sim zone fits the
          // window

  assert(scale > 0 && std::isfinite(scale));

  double graph_radius = scale * sim_radius;
  double close_radius = 300.;
  double sep_radius = 25.;

  TApplication app = TApplication("Window", (int*)0, (char**)nullptr);

  TCanvas histo_canvas{"canvas", "Boid statistics", -1, 0, 600, 600};

  // initialization of graphical features
  sf::RenderWindow window(sf::VideoMode(window_x, window_y),
                          "Boid Simulation");  // render window
  window.setPosition(sf::Vector2i(1350, 0));
  sf::CircleShape sim_zone(graph_radius + 20.,
                           100.);  // simulation area, the "sky"
  sim_zone.setPosition(
      -graph_radius - 20.,
      -graph_radius - 20.);  // put sim_zone center at coords (0,0)
  sf::RectangleShape background(
      sf::Vector2f(window_x, window_y));  // background rectangle
  background.setPosition(-window_x / 2.,
                         -window_y / 2.);  // center background at coords (0,0)
  background.setFillColor(
      sf::Color(240, 240, 240));  // background color: off-white
  sim_zone.setFillColor(sf::Color(50, 150, 255));  // sim_zone color: light blue
  // center view on (0,0)
  sf::View view;
  view.reset(sf::FloatRect(-window_x / 2., -window_y / 2., window_x, window_y));
  view.setViewport(sf::FloatRect(0.f, 0.f, 1.f, 1.f));
  window.setView(view);

  // initialization of boids and predators
  std::vector<Boid> boids;
  std::vector<Boid> predators;
  std::vector<sf::CircleShape> boid_triangles;
  std::vector<sf::CircleShape> pred_triangles;
  std::vector<sw::Ring> boid_sights;
  std::vector<sw::Ring> pred_sights;

  std::srand(static_cast<unsigned int>(
      std::time(nullptr)));  // seed RNG with system time

  // spawn boids
  std::vector<sf::Color> flock_colors;
  for (int i = 0; i < n_flocks; ++i) {
    sf::Uint8 flock_color_r = (std::rand() % 5) * 40;
    sf::Uint8 flock_color_g = (std::rand() % 5) * 40;
    sf::Uint8 flock_color_b = (std::rand() % 5) * 40;
    flock_colors.push_back(
        sf::Color(flock_color_r, flock_color_g, flock_color_b));
    for (int j = 0; j < flock_pops[i]; ++j) {
      double spawn_radius = (0.5 * sim_radius) * std::rand() / RAND_MAX;
      Angle spawn_angle{360. * std::rand() / RAND_MAX};
      Position spawn_pos{spawn_radius * spawn_angle.getCosine(),
                         spawn_radius * spawn_angle.getSine()};
      double spawn_speed =
          std::sqrt(1E-12 * MAX_SPEED2) * std::rand() / (RAND_MAX);
      Angle spawnspeed_angle{360. * std::rand() / RAND_MAX};
      Velocity spawn_vel{spawn_speed * spawn_angle.getCosine(),
                         spawn_radius * spawn_angle.getSine()};
      boids.push_back(Boid(spawn_pos, spawn_vel, Angle(0.), i));

      initializeGraphic(
          boids[i], sf::CircleShape(9., 3.), boid_triangles, boid_sights,
          sight_angle, flock_colors.back(),
          sf::Color(flock_color_r, flock_color_g, flock_color_b, 90), window,
          scale);
    }
  }
  assert(boids.size() == static_cast<unsigned long>(n_boids));

  // spawn predators
  for (int i = 0; i < n_preds; ++i) {
    double spawn_radius = (0.5 * sim_radius) * std::rand() /
                          RAND_MAX;  // change names, these are polar coords
    Angle spawn_angle{360. * std::rand() / RAND_MAX};
    Position spawn_pos{spawn_radius * spawn_angle.getCosine(),
                       spawn_radius * spawn_angle.getSine()};
    double spawn_speed =
        std::sqrt(1E-12 * MAX_SPEED2) * std::rand() / (RAND_MAX);
    Angle spawnspeed_angle{360. * std::rand() / RAND_MAX};
    Velocity spawn_vel{spawn_speed * spawn_angle.getCosine(),
                       spawn_radius * spawn_angle.getSine()};
    predators.push_back(Boid(spawn_pos, spawn_vel));

    initializeGraphic(predators[i], sf::CircleShape(11., 3.), pred_triangles,
                      pred_sights, sight_angle, sf::Color(200, 50, 50),
                      sf::Color(100, 25, 25, 50), window, scale);
  }
  assert(predators.size() == static_cast<unsigned long>(n_preds));

  int iteration = 0;  // iteration counter
  int printing = 1;
  int update_time_ms = 16;  // time between updates, in milliseconds
  int print_sep_ms = 5000;  // time between stat prints, in milliseconds

  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }

    std::vector<Boid> future_boids = boids;
    for (int i = 0; i < n_boids; ++i) {
      future_boids[i].updateBoidVelocity(boids, predators, close_radius,
                                         sep_radius, sep_factor, align_factor,
                                         cohes_factor, sight_angle);
      future_boids[i].moveBoid(update_time_ms / 1000.);
    }

    std::vector<Boid> future_predators = predators;
    for (int i = 0; i < n_preds; ++i) {
      future_predators[i].updatePredatorVelocity(
          predators, boids, close_radius, sep_radius, sep_factor, align_factor,
          cohes_factor, sight_angle);
      future_predators[i].moveBoid(update_time_ms /
                                   1000.);  // moveBoid takes seconds
    }
    boids = future_boids;
    predators = future_predators;

    window.clear();
    window.draw(background);
    window.draw(sim_zone);
    for (int i = 0; i < n_boids; ++i) {
      updateGraphic(boids[i], boid_triangles[i], boid_sights[i], window, scale);
    }
    for (int i = 0; i < n_preds; ++i) {
      updateGraphic(predators[i], pred_triangles[i], pred_sights[i], window,
                    scale);
    }
    window.display();

    if (iteration % (print_sep_ms / update_time_ms) ==
        0) {  // print roughly every print_sep_ms milliseconds

      histo_canvas.Clear();
      histo_canvas.Divide(2, 2);

      std::vector<std::vector<double>> flock_dists;
      for (int i = 0; i < n_flocks; ++i)
        flock_dists.push_back(getFlockDistances(boids, i));

      std::vector<std::vector<double>> flock_speeds;
      for (int i = 0; i < n_flocks; ++i)
        flock_speeds.push_back(getFlockSpeeds(boids, i));

      std::vector<std::vector<double>> predflock_dists;
      for (int i = 0; i < n_flocks; ++i)
        predflock_dists.push_back(getPredFlockDistances(boids, i, predators));

      std::vector<double> pred_speeds = getFlockSpeeds(predators, 0);

      histo_canvas.cd(1);
      TH1D dist_scale_canvas = TH1D("", "", 50, 0., std::sqrt(MAX_RADIUS2));
      dist_scale_canvas.SetMaximum(
          (n_boids * n_boids) /
          (15 * n_flocks));  // considering bins less important, as boids get
                             // close easily
      gStyle->SetOptStat(0);
      dist_scale_canvas.UseCurrentStyle();
      dist_scale_canvas.DrawCopy("AXIS");
      std::vector<TH1D> flock_dist_histos;
      for (int i = 0; i < n_flocks; ++i) {
        std::string h_base_begin = "Flock ";
        std::string h_flock_number = std::to_string(i + 1);
        std::string h_base_end = " dists";
        std::string h_name = h_base_begin + h_flock_number + h_base_end;
        flock_dist_histos.push_back(TH1D(h_name.c_str(), "Flock distances", 50,
                                         0., std::sqrt(MAX_RADIUS2)));
        fillDrawHisto(flock_dist_histos[i], flock_dists[i],
                      TColor::GetColor(flock_colors[i].r, flock_colors[i].g,
                                       flock_colors[i].b),
                      i);
      }

      histo_canvas.cd(2);
      TH1D speed_scale_canvas =
          TH1D("", "", 50, 0., 0.01 * std::sqrt(MAX_SPEED2));
      gStyle->SetOptStat(0);
      speed_scale_canvas.UseCurrentStyle();
      speed_scale_canvas.SetMaximum(n_boids / n_flocks);
      speed_scale_canvas.DrawCopy("AXIS");
      std::vector<TH1D> flock_speed_histos;
      for (int i = 0; i < n_flocks; ++i) {
        std::string h_base_begin = "Flock ";
        std::string h_flock_number = std::to_string(i + 1);
        std::string h_base_end = " speeds";
        std::string h_name = h_base_begin + h_flock_number + h_base_end;
        flock_speed_histos.push_back(TH1D(h_name.c_str(), "Flock speeds", 50,
                                          0., 0.01 * std::sqrt(MAX_SPEED2)));
        fillDrawHisto(flock_speed_histos[i], flock_speeds[i],
                      TColor::GetColor(flock_colors[i].r, flock_colors[i].g,
                                       flock_colors[i].b),
                      i);
      }

      histo_canvas.cd(3);
      dist_scale_canvas.SetMaximum(n_boids / n_flocks);
      gStyle->SetOptStat(0);
      dist_scale_canvas.UseCurrentStyle();
      dist_scale_canvas.DrawCopy("AXIS");
      std::vector<TH1D> predflock_dist_histos;
      for (int i = 0; i < n_flocks; ++i) {
        std::string h_base_begin = "Flock ";
        std::string h_flock_number = std::to_string(i + 1);
        std::string h_base_end = " pred dists";
        std::string h_name = h_base_begin + h_flock_number + h_base_end;
        predflock_dist_histos.push_back(TH1D(h_name.c_str(),
                                             "Flock distances from predators",
                                             50, 0., std::sqrt(MAX_RADIUS2)));
        fillDrawHisto(predflock_dist_histos[i], predflock_dists[i],
                      TColor::GetColor(flock_colors[i].r, flock_colors[i].g,
                                       flock_colors[i].b),
                      i);
      }

      histo_canvas.cd(4);
      speed_scale_canvas.SetMaximum(n_preds);
      gStyle->SetOptStat(0);
      speed_scale_canvas.UseCurrentStyle();
      speed_scale_canvas.DrawCopy("AXIS");
      TH1D pred_speed_histo = TH1D("Pred speeds", "Predator speeds", 50, 0.,
                                   0.01 * std::sqrt(MAX_SPEED2));
      fillDrawHisto(pred_speed_histo, pred_speeds, Color_t{2}, 0.f);

      std::cout << "\n==================== ITERATION " << iteration
                << " ====================\n";
      for (int i = 0; i < n_flocks; ++i) {
        std::cout << "Avg distance of flock " << i + 1 << ": "
                  << flock_dist_histos[i].GetMean()
                  << "\t Std dev: " << flock_dist_histos[i].GetStdDev() << "\n";
      }
      for (int i = 0; i < n_flocks; ++i) {
        std::cout << "Avg speed of flock " << i + 1 << ": "
                  << flock_speed_histos[i].GetMean()
                  << "\t Std dev: " << flock_speed_histos[i].GetStdDev()
                  << "\n";
      }
      for (int i = 0; i < n_flocks; ++i) {
        std::cout << "Avg predator distance from flock " << i + 1 << ": "
                  << predflock_dist_histos[i].GetMean()
                  << "\t Std dev: " << predflock_dist_histos[i].GetStdDev()
                  << "\n";
      }
      std::cout << "Avg speed of predators: " << pred_speed_histo.GetMean()
                << "\t Std dev: " << pred_speed_histo.GetStdDev() << "\n";

      std::filesystem::create_directory("./Histograms");
      std::string histo_root = "Histograms/Histo";
      char begintime_string[50];
      strftime(begintime_string, 25, "%y%m%d_%H%M%S_", bgn_tm);
      std::string histo_time = begintime_string;
      std::string histo_printing = std::to_string(printing);
      std::string histo_format = ".pdf";
      std::string histo_name =
          histo_root + histo_time + histo_printing + histo_format;
      histo_canvas.Print(histo_name.c_str());

      ++printing;
    }

    ++iteration;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(update_time_ms));  // moveBoid takes seconds
  }
}