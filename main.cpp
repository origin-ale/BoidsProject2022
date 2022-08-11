#include "main.hpp"

int main()
{
  std::vector<Boid> boids;
  int iterations = 1E5;
  int n_boids=10; //number of boids, to enter in input
  if(n_boids<=0 || !(std::isfinite(n_boids))) throw E_InvalidNumberOfBoids{};
  double close_radius = 100.;
  double sep_radius = 1; 
  double sep_factor = 100.; 
  double align_factor = 0.5;
  double cohes_factor = 0.5;

  double sim_radius = std::sqrt(MAX_RADIUS2);
  std::srand(static_cast<unsigned int>(std::time(nullptr)));
  for(int i=0; i<n_boids; ++i) {
    double spawn_radius = sim_radius * std::rand()/RAND_MAX;
    Angle spawn_angle{360. * std::rand()/RAND_MAX};
    Position spawn_pos{spawn_radius * spawn_angle.getCosine(), spawn_radius * spawn_angle.getSine()};
    boids.push_back(Boid(spawn_pos));
  }

  assert(boids.size() == static_cast<unsigned long>(n_boids));

  for(int i = 0; i <= iterations; ++i){
    //DEBUGGING TOOL
    //   std::cout << "---------- ITERATION " << i << " ----------\n";
    //   for(int j=0; j< 10; ++j) {
    //     std::cout << "Position of boid " << j << ": " << "(" << boids[j].getPosition().getX() << "," << boids[j].getPosition().getY() << ")\n";
    //     std::cout << "Velocity of boid " << j << ": " << "(" << boids[j].getVelocity().getXVel() << "," << boids[j].getVelocity().getYVel() << ")\n";
    // }
    std::vector<Boid> future_boids = boids;
    for(int i=0; i<n_boids; ++i) {
      future_boids[i].updateVelocity(boids, close_radius, sep_radius, sep_factor, align_factor, cohes_factor);
      future_boids[i].moveBoid(TIME_STEP);
    }
    boids = future_boids;
  

    if(i % 5000 == 0){ //maybe implement in stats file?
      std::vector<double> distances;
      std::vector<double> speeds;
      for(int j = 0; j < n_boids; ++j){
        for(int k = 0; k < n_boids; ++k){
          if(j!=k) distances.push_back(sqrt((boids[j].getPosition() - boids[k].getPosition()).getNorm2()));
        }
        speeds.push_back(sqrt(boids[j].getVelocity().getNorm2()));
      }

    double dist_sum = std::accumulate(distances.begin(), distances.end(), 0.0); //ABSOLUTELY IMPLEMENT AS FUNCTIONS
    double dist_mean = dist_sum / distances.size();
    std::vector<double> dist_diff(distances.size());
    std::transform(distances.begin(), distances.end(), dist_diff.begin(), [dist_mean](double x) { return x - dist_mean; });
    double dist_sq_sum = std::inner_product(dist_diff.begin(), dist_diff.end(), dist_diff.begin(), 0.0);
    double dist_stdev = std::sqrt(dist_sq_sum / distances.size());

    double speed_sum = std::accumulate(speeds.begin(), speeds.end(), 0.0);
    double speed_mean = speed_sum / speeds.size();
    std::vector<double> speed_diff(speeds.size());
    std::transform(speeds.begin(), speeds.end(), speed_diff.begin(), [speed_mean](double x) { return x - speed_mean; });
    double speed_sq_sum = std::inner_product(speed_diff.begin(), speed_diff.end(), speed_diff.begin(), 0.0);
    double speed_stdev = std::sqrt(speed_sq_sum / speeds.size());

    std::cout << "---------- ITERATION " << i << " ----------\n"
              << "Average distance between boids: " << dist_mean << "\tStandard deviation: " << dist_stdev << "\n" //completely messed up
              << "Average speed of boids: " << speed_mean << "\tStandard deviation: " << speed_stdev << "\n\n"; 
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