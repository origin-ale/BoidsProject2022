#include "main.hpp"

int main()
{
  int n_boids;
  try{
    std::cout << "Enter number of boids: "; std::cin >> n_boids;
  } catch(E_InvalidNumberOfBoids) { while (n_boids<=0 || !(std::isfinite(n_boids))) {
    std::cout << "Invalid number of boids entered. Please enter again: ";  std::cin >> n_boids; }
  }

  double sep_factor;
  try{
    std::cout << "Enter separation factor: "; std::cin >> sep_factor;
  } catch(E_InvalidSeparationFactor) { while (sep_factor<0 || !(std::isfinite(sep_factor))) {
    std::cout << "Invalid separation factor entered. Please enter again: ";  std::cin >> sep_factor; }
  }

  double align_factor;
  try{
    std::cout << "Enter alignment factor: "; std::cin >> align_factor;
  } catch(E_InvalidAlignmentFactor) { while (align_factor<0 || align_factor>=1 || !(std::isfinite(align_factor))) {
    std::cout << "Invalid alignment factor entered. Please enter again: ";  std::cin >> align_factor; }
  }

  double cohes_factor;
  try{
    std::cout << "Enter cohesion factor: "; std::cin >> cohes_factor;
  } catch(E_InvalidCohesionFactor) { while (cohes_factor<0 || !(std::isfinite(cohes_factor))) {
    std::cout << "Invalid cohesion factor entered. Please enter again: ";  std::cin >> cohes_factor; }
  }

  double n_preds;
  try{
    std::cout << "Enter number of predators: "; std::cin >> n_preds;
  } catch(E_InvalidNumberOfBoids) { while (n_preds<0 || !(std::isfinite(n_preds))) {
    std::cout << "Invalid number of predators entered. Please enter again: ";  std::cin >> n_preds; }
  }

  double sight_angle;
  try{
    std::cout << "Enter sight angle: "; std::cin >> sight_angle;
  } catch(E_InvalidSightAngle) { while (sight_angle<0 || sight_angle>180 || !(std::isfinite(sight_angle))) {
    std::cout << "Invalid sight_angle entered. Please enter again: ";  std::cin >> sight_angle; }
  }

  double close_radius = 100.;
  double sep_radius = 1; 
  std::vector<Boid> boids;
  std::vector<Boid> predators;
  int iterations = 1E5;
  double sim_radius = std::sqrt(MAX_RADIUS2);

  std::srand(static_cast<unsigned int>(std::time(nullptr)));
  for(int i=0; i<n_boids; ++i) {
    double spawn_radius = sim_radius * std::rand()/RAND_MAX;
    Angle spawn_angle{360. * std::rand()/RAND_MAX};
    Position spawn_pos{spawn_radius * spawn_angle.getCosine(), spawn_radius * spawn_angle.getSine()};
    boids.push_back(Boid(spawn_pos));
  }
  assert(boids.size() == static_cast<unsigned long>(n_boids));

  std::srand(static_cast<unsigned int>(std::time(nullptr)));
  for(int i=0; i<n_preds; ++i) {
    double spawn_radius = sim_radius * std::rand()/RAND_MAX;
    Angle spawn_angle{360. * std::rand()/RAND_MAX};
    Position spawn_pos{spawn_radius * spawn_angle.getCosine(), spawn_radius * spawn_angle.getSine()};
    predators.push_back(Boid(spawn_pos));
  }
  assert(predators.size() == static_cast<unsigned long>(n_preds));

  for(int i = 0; i <= iterations; ++i){
    //DEBUGGING TOOL
    //   std::cout << "---------- ITERATION " << i << " ----------\n";
    //   for(int j=0; j< 10; ++j) {
    //     std::cout << "Position of boid " << j << ": " << "(" << boids[j].getPosition().getX() << "," << boids[j].getPosition().getY() << ")\n";
    //     std::cout << "Velocity of boid " << j << ": " << "(" << boids[j].getVelocity().getXVel() << "," << boids[j].getVelocity().getYVel() << ")\n";
    // }
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

    if(i % 5000 == 0){ //maybe implement in stats file?

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

    std::cout << "---------- ITERATION " << i << " ----------\n"
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
  }
}