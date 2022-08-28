#include "main.hpp"

int main()
{
  bool debug = true; //debug flag, skips parameter input

  int n_flocks;
  std::vector<int> flock_pops;
  int n_boids;
  double sep_factor;
  double align_factor;
  double cohes_factor;
  int n_preds;
  double sight_angle;

  if(debug){
    n_flocks = 3;
    flock_pops = {34,33,33};
    sep_factor = 0.5;
    align_factor = 0.1;
    cohes_factor = 0.3;
    n_preds = 2;
    sight_angle = 60.;
  }

  else
  {
    //input parameters
    std::cout << "Enter number of flocks: "; 
    std::cin >> n_flocks;
    while(n_flocks<0 || !(std::isfinite(n_flocks))){
      std::cout << "Invalid number of flocks entered. Please enter again: ";  
      std::cin >> n_flocks; 
    }

    for(int i=0; i<n_flocks; ++i){
      std::cout << "Enter number of boids in flock " << i+1 << ": ";
      int input;
      std:: cin >> input;
      flock_pops.push_back(input);
      while(flock_pops[i]<0 || !(std::isfinite(flock_pops[i]))){
        std::cout << "Invalid number of boids entered. Please enter again: ";  
        std::cin >> flock_pops[i]; 
    }
    }
    
    std::cout << "Enter separation factor: "; 
    std::cin >> sep_factor; //recommended values around 0.5
    while(sep_factor<0 || !(std::isfinite(sep_factor))) {
      std::cout << "Invalid separation factor entered. Please enter again: ";  
      std::cin >> sep_factor; 
    }
    
    std::cout << "Enter alignment factor: "; 
    std::cin >> align_factor; //recommended values around 0.1
    while(align_factor<0 || align_factor>=1 || !(std::isfinite(align_factor))) {
        std::cout << "Invalid alignment factor entered. Please enter again: ";  
        std::cin >> align_factor; 
    }

    std::cout << "Enter cohesion factor: "; 
    std::cin >> cohes_factor; //recommended values around 0.3
    while(cohes_factor<0 || !(std::isfinite(cohes_factor))) {
      std::cout << "Invalid cohesion factor entered. Please enter again: ";  
      std::cin >> cohes_factor; 
    }
    
    std::cout << "Enter number of predators: "; 
    std::cin >> n_preds;
    while (n_preds<0 || !(std::isfinite(n_preds))) {
      std::cout << "Invalid number of predators entered. Please enter again: ";  
      std::cin >> n_preds; 
    }

    std::cout << "Enter sight angle: "; 
    std::cin >> sight_angle; //recommended values around 0.3
    while(cohes_factor<0 || !(std::isfinite(sight_angle))) {
      std::cout << "Invalid sight angle entered. Please enter again: ";  
      std::cin >> sight_angle; 
    }
  }

  n_boids = std::accumulate(flock_pops.begin(), flock_pops.end(), 0);

  //parameter assertions
  assert(n_boids>=0 && std::isfinite(n_boids));
  assert(sep_factor>=0 && std::isfinite(sep_factor));
  assert(align_factor>=0 && align_factor < 1 && std::isfinite(align_factor));
  assert(cohes_factor>=0 && std::isfinite(cohes_factor));
  assert(n_preds>=0 && std::isfinite(n_preds));
  assert(sight_angle<=180 && std::isfinite(sight_angle));

  //fixed parameter initialization
  int window_x = 1500;
  int window_y = 1500;
  double sim_radius = std::sqrt(MAX_RADIUS2);
  double scale = (std::min(window_x - 50. , window_y - 50.) / sim_radius) /2; //coord-to-pixel scaling factor, such that the sim zone fits the window //replace numbers with something proportional to boid sprite size
  double graph_radius = scale * sim_radius;
  double close_radius = 150.;
  double sep_radius = 25.;

  //initialization of graphical features
  sf::RenderWindow window(sf::VideoMode(window_x, window_y), "Boid Simulation"); //render window
  window.setPosition(sf::Vector2i(1350, 0));
  sf::CircleShape sim_zone(graph_radius+20., 100.); //simulation area, the "sky"
  sim_zone.setPosition(-graph_radius-20., -graph_radius-20.); //put sim_zone center at coords (0,0)
  sf::RectangleShape background(sf::Vector2f(window_x, window_y)); //background rectangle
  background.setPosition(-window_x/2., -window_y/2.); //center background at coords (0,0)
  background.setFillColor(sf::Color(240,240,240)); //background color: off-white
  sim_zone.setFillColor(sf::Color(50,150,255)); //sim_zone color: light blue
  //center view on (0,0)
  sf::View view;
  view.reset(sf::FloatRect(-window_x/2., -window_y/2., window_x, window_y));
  view.setViewport(sf::FloatRect(0.f, 0.f, 1.f, 1.f));
  window.setView(view);

  //initialization of boids and predators
  std::vector<Boid> boids;
  std::vector<Boid> predators;
  std::vector<sf::CircleShape> boid_triangles;
  std::vector<sf::CircleShape> pred_triangles;
  std::vector<sw::Ring> boid_sights;
  std::vector<sw::Ring> pred_sights;

  TApplication app = TApplication("Window", (int*)0, (char**)nullptr);
  
  TCanvas histo_canvas{"canvas", "Boid statistics", - 1, 0, 600, 600};
  histo_canvas.Divide(2,2);


  std::srand(static_cast<unsigned int>(std::time(nullptr))); //seed RNG with system time

  //spawn boids
  for(int i=0; i<n_flocks; ++i) {
    sf::Uint8 flock_color_r = (std::rand()%5) *40;
    sf::Uint8 flock_color_g = (std::rand()%5) *40;
    sf::Uint8 flock_color_b = (std::rand()%5) *40;
    sf::Color flock_color{flock_color_r, flock_color_g, flock_color_b};
    for(int j=0; j<flock_pops[i]; ++j){
      double spawn_radius = (0.5 * sim_radius) * std::rand()/RAND_MAX; //change names, these are polar coords
      Angle spawn_angle{360. * std::rand()/RAND_MAX};
      Position spawn_pos{spawn_radius * spawn_angle.getCosine(), spawn_radius * spawn_angle.getSine()};
      double spawn_speed = std::sqrt(1E-12 * MAX_SPEED2) * std::rand()/(RAND_MAX);
      Angle spawnspeed_angle{360. * std::rand()/RAND_MAX};
      Velocity spawn_vel{spawn_speed * spawn_angle.getCosine(), spawn_radius * spawn_angle.getSine()};
      boids.push_back(Boid(spawn_pos,spawn_vel, Angle(0.), i));

      initializeGraphic(boids[i], sf::CircleShape(9.,3.), boid_triangles, boid_sights, sight_angle, flock_color, sf::Color(flock_color_r, flock_color_g, flock_color_b, 90), window, scale);
    }
  }
  assert(boids.size() == static_cast<unsigned long>(n_boids));

  //spawn predators
  for(int i=0; i<n_preds; ++i) {
    double spawn_radius = (0.5 * sim_radius) * std::rand()/RAND_MAX; //change names, these are polar coords
    Angle spawn_angle{360. * std::rand()/RAND_MAX};
    Position spawn_pos{spawn_radius * spawn_angle.getCosine(), spawn_radius * spawn_angle.getSine()};
    double spawn_speed = std::sqrt(1E-12 * MAX_SPEED2) * std::rand()/(RAND_MAX);
    Angle spawnspeed_angle{360. * std::rand()/RAND_MAX};
    Velocity spawn_vel{spawn_speed * spawn_angle.getCosine(), spawn_radius * spawn_angle.getSine()};
    predators.push_back(Boid(spawn_pos,spawn_vel));

    initializeGraphic(predators[i], sf::CircleShape(11.,3.), pred_triangles, pred_sights, sight_angle, sf::Color(200, 50, 50), sf::Color(100, 25, 25, 50), window, scale);
  }
  assert(predators.size() == static_cast<unsigned long>(n_preds));

  int iteration = 0; //iteration counter
  int update_time_ms = 16; //time between updates, in milliseconds
  int print_sep_ms = 5000; //time between stat prints, in milliseconds

  while (window.isOpen())
  {
    sf::Event event;
    while (window.pollEvent(event))
    {
      if (event.type == sf::Event::Closed){
        window.close();
      }
    }
    gSystem->ProcessEvents();

    std::vector<Boid> future_boids = boids;
    for(int i=0; i<n_boids; ++i) {
      future_boids[i].updateBoidVelocity(boids, predators, close_radius, sep_radius, sep_factor, align_factor, cohes_factor, sight_angle);
      future_boids[i].moveBoid(TIME_STEP);
    }
    boids = future_boids;

    std::vector<Boid> future_predators = predators;
    for(int i=0; i<n_preds; ++i) {
      future_predators[i].updatePredatorVelocity(predators, boids, close_radius, sep_radius, sep_factor, align_factor, cohes_factor, sight_angle);
      future_predators[i].moveBoid(TIME_STEP);
    }
    predators = future_predators;

    window.clear();
    window.draw(background);
    window.draw(sim_zone);
    for(int i = 0; i < n_boids; ++i){
      updateGraphic(boids[i], boid_triangles[i], boid_sights[i], window, scale);
    }
    for(int i = 0; i < n_preds; ++i){
      updateGraphic(predators[i], pred_triangles[i], pred_sights[i], window, scale);
    }
    window.display();

    std::this_thread::sleep_for(std::chrono::milliseconds(update_time_ms));

    if(iteration % (print_sep_ms / update_time_ms) == 0){ //print roughly every print_sep_ms milliseconds
      std::vector<double> boid_distances = getDistances(boids, boids);
      std::vector<double> boid_speeds = getSpeeds(boids);
      std::vector<double> boidpred_distances = getDistances(boids, predators);
      std::vector<double> pred_speeds = getSpeeds(predators);

      Stats boid_dist_stats = getStats(boid_distances);
      Stats boid_speed_stats = getStats(boid_speeds);
      Stats boidpred_dist_stats = getStats(boidpred_distances);
      Stats pred_speed_stats = getStats(pred_speeds);

      std::cout << "---------- ITERATION " << iteration << " ----------\n"
                << "Average boid - boid distance: " << boid_dist_stats.mean << "\t\tStandard deviation: " << boid_dist_stats.stdev << "\n"
                << "Average boid speed: " << boid_speed_stats.mean << "\t\t\tStandard deviation: " << boid_speed_stats.stdev << "\n"
                << "Average boid - predator distance: " << boidpred_dist_stats.mean << "\tStandard deviation: " << boid_dist_stats.stdev << "\n"
                << "Average predator speed: " << pred_speed_stats.mean << "\t\t\tStandard deviation: " << pred_speed_stats.stdev << "\n\n"; 
                
      makeHisto(boid_distances, "Boid-boid distances", "Distance stats", std::sqrt(MAX_RADIUS2), histo_canvas, 1);
      makeHisto(boid_speeds, "Boid speeds", "Speed stats", 0.01 * std::sqrt(MAX_SPEED2), histo_canvas, 2);
      makeHisto(boidpred_distances, "Boid-predator distances", "Distance stats", std::sqrt(MAX_RADIUS2), histo_canvas, 3);
      makeHisto(pred_speeds, "Predator speeds", "Speed stats", 0.01 * std::sqrt(MAX_SPEED2), histo_canvas, 4);

      histo_canvas.Print("latest_histo.pdf");
    }

    ++iteration;
  }
}