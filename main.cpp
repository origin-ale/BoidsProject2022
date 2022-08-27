#include "main.hpp"

int main()
{
  bool debug = true; //debug flag, skips parameter input

  int n_boids;
  double sep_factor;
  double align_factor;
  double cohes_factor;
  int n_preds;
  double sight_angle;

  if(debug){
    n_boids = 50;
    sep_factor = 0.5;
    align_factor = 0.1;
    cohes_factor = 0.3;
    n_preds = 2;
    sight_angle = 60.;
  }

  else
  {
    //input parameters
    std::cout << "Enter number of boids: "; 
    std::cin >> n_boids;
    while(n_boids<0 || !(std::isfinite(n_boids))){
      std::cout << "Invalid number of boids entered. Please enter again: ";  
      std::cin >> n_boids; 
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

  //parameter assertions
  assert(n_boids>=0 && std::isfinite(n_boids));
  assert(sep_factor>=0 && std::isfinite(sep_factor));
  assert(align_factor>=0 && align_factor < 1 && std::isfinite(align_factor));
  assert(cohes_factor>=0 && std::isfinite(cohes_factor));
  assert(n_preds>=0 && std::isfinite(n_preds));
  assert(sight_angle<=180 && std::isfinite(sight_angle));

  //fixed parameter initialization
  double window_x = 1500.;
  double window_y = 1500.;
  double sim_radius = std::sqrt(MAX_RADIUS2);
  double scale = (std::min(window_x - 50. , window_y - 50.) / sim_radius) /2; //coord-to-pixel scaling factor, such that the sim zone fits the window //replace numbers with something proportional to boid sprite size
  double graph_radius = scale * sim_radius;
  double close_radius = 150.;
  double sep_radius = 25.;

  //initialization of graphical features
  sf::RenderWindow window(sf::VideoMode(window_x, window_y), "Boids Simulation"); //render window
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

  std::srand(static_cast<unsigned int>(std::time(nullptr))); //seed RNG with system time

  //spawn boids
  for(int i=0; i<n_boids; ++i) {
    double spawn_radius = (0.5 * sim_radius) * std::rand()/RAND_MAX; //change names, these are polar coords
    Angle spawn_angle{360. * std::rand()/RAND_MAX};
    Position spawn_pos{spawn_radius * spawn_angle.getCosine(), spawn_radius * spawn_angle.getSine()};
    double spawn_speed = std::sqrt(1E-12 * MAX_SPEED2) * std::rand()/(RAND_MAX);
    Angle spawnspeed_angle{360. * std::rand()/RAND_MAX};
    Velocity spawn_vel{spawn_speed * spawn_angle.getCosine(), spawn_radius * spawn_angle.getSine()};
    boids.push_back(Boid(spawn_pos,spawn_vel));

    initializeGraphic(boids[i], sf::CircleShape(9.,3.), boid_triangles, boid_sights, sight_angle, sf::Color::Black, sf::Color(30, 200 ,30, 90), window, scale);
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
      if (event.type == sf::Event::Closed)
        window.close();
    }

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
    //TEST--------
    // sf::CircleShape test_triangle{100.,3};
    // sf::CircleShape marker{10.};

    // test_triangle.setOrigin(100.,100.);
    // test_triangle.setPosition(0.,0.);

    // marker.setFillColor(sf::Color::Red);
    // marker.setOrigin(10.,10.);
    // marker.setPosition(test_triangle.getPosition());

    // //std::cout<< test_triangle.getPosition().x << " " << test_triangle.getPosition().y;

    // window.draw(test_triangle);
    // window.draw(marker);
    //-------------
    for(int i = 0; i < n_boids; ++i){
      updateGraphic(boids[i], boid_triangles[i], boid_sights[i], window, scale);
    }
    for(int i = 0; i < n_preds; ++i){
      updateGraphic(predators[i], pred_triangles[i], pred_sights[i], window, scale);
    }
    window.display();
    std::this_thread::sleep_for(std::chrono::milliseconds(update_time_ms));
    //std::cout << "pos: (" << boids[0].getPosition().getX() << ", " << boids[0].getPosition().getY() << "); vel: (" << boids[0].getVelocity().getXVel() << ", " << boids[0].getVelocity().getYVel() << ")\n";

    if(iteration % (print_sep_ms / update_time_ms) == 0){ //print roughly every print_sep_ms milliseconds
    //maybe implement in stats file?

    //ordinary boids statistics
    std::vector<double> b_distances;
    std::vector<double> b_speeds;
    for(int j = 0; j < n_boids; ++j){
      for(int k = j+1; k < n_boids; ++k){
        b_distances.push_back(sqrt((boids[j].getPosition() - boids[k].getPosition()).getNorm2()));
      }
      b_speeds.push_back(sqrt(boids[j].getVelocity().getNorm2())); }
    
    double b_dist_sum = std::accumulate(b_distances.begin(), b_distances.end(), 0.0); //ABSOLUTELY IMPLEMENT AS FUNCTIONS
    double b_dist_mean = b_dist_sum / b_distances.size();
    std::vector<double> b_dist_diff(b_distances.size());
    std::transform(b_distances.begin(), b_distances.end(), b_dist_diff.begin(), [b_dist_mean](double x) { return x - b_dist_mean; });
    double b_dist_sq_sum = std::inner_product(b_dist_diff.begin(), b_dist_diff.end(), b_dist_diff.begin(), 0.0);
    double b_dist_stdev = std::sqrt(b_dist_sq_sum / b_distances.size());

    double b_speed_sum = std::accumulate(b_speeds.begin(), b_speeds.end(), 0.0);
    double b_speed_mean = b_speed_sum / b_speeds.size();
    std::vector<double> b_speed_diff(b_speeds.size());
    std::transform(b_speeds.begin(), b_speeds.end(), b_speed_diff.begin(), [b_speed_mean](double x) { return x - b_speed_mean; });
    double b_speed_sq_sum = std::inner_product(b_speed_diff.begin(), b_speed_diff.end(), b_speed_diff.begin(), 0.0);
    double b_speed_stdev = std::sqrt(b_speed_sq_sum / b_speeds.size());

    //predators statistics
    std::vector<double> p_distances;
    std::vector<double> p_speeds;
    for(int j = 0; j < n_preds; ++j){
      for(int k = 0; k < n_boids; ++k){
        p_distances.push_back(sqrt((predators[j].getPosition() - boids[k].getPosition()).getNorm2()));
      }
      p_speeds.push_back(sqrt(predators[j].getVelocity().getNorm2())); }

    double p_dist_sum = std::accumulate(p_distances.begin(), p_distances.end(), 0.0); //ABSOLUTELY IMPLEMENT AS FUNCTIONS
    double p_dist_mean = p_dist_sum / p_distances.size();
    std::vector<double> p_dist_diff(p_distances.size());
    std::transform(p_distances.begin(), p_distances.end(), p_dist_diff.begin(), [p_dist_mean](double x) { return x - p_dist_mean; });
    double p_dist_sq_sum = std::inner_product(p_dist_diff.begin(), p_dist_diff.end(), p_dist_diff.begin(), 0.0);
    double p_dist_stdev = std::sqrt(p_dist_sq_sum / p_distances.size());
    
    double p_speed_sum = std::accumulate(p_speeds.begin(), p_speeds.end(), 0.0);
    double p_speed_mean = p_speed_sum / p_speeds.size();
    std::vector<double> p_speed_diff(p_speeds.size());
    std::transform(p_speeds.begin(), p_speeds.end(), p_speed_diff.begin(), [p_speed_mean](double x) { return x - p_speed_mean; });
    double p_speed_sq_sum = std::inner_product(p_speed_diff.begin(), p_speed_diff.end(), p_speed_diff.begin(), 0.0);
    double p_speed_stdev = std::sqrt(p_speed_sq_sum / p_speeds.size());

    std::cout << "---------- ITERATION " << iteration << " ----------\n"
              << "Average distance between boids: " << b_dist_mean << "\tStandard deviation: " << b_dist_stdev << "\n" //completely messed up
              << "Average speed of boids: " << b_speed_mean << "\tStandard deviation: " << b_speed_stdev << "\n\n"
              << "Average distance between boids and predators: " << p_dist_mean << "\tStandard deviation: " << p_dist_stdev << "\n"
              << "Average speed of predators: " << p_speed_mean << "\tStandard deviation: " << p_speed_stdev << "\n\n"; 
    // std::cout << "BOID POSITIONS:\n";
    // for(auto item: boids){
    //   std::cout << item.getPosition().getX() << " , " << item.getPosition().getY() << "\n";
    // }
    
    // std::cout << "DISTANCES:\n";
    // for(auto item: distances){
    //   std::cout << item << "\n";
    // }
    }
  ++ iteration;
  }
}