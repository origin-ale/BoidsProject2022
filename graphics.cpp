#include "main.hpp"
#include "graphics.hpp"

int main()
{
    sf::RenderWindow window(sf::VideoMode(2400, 1500), "Boids Simulation");
    sf::CircleShape sim_zone(750., 100.);
    sim_zone.setPosition(-750.,-750.);
    sf::RectangleShape background(sf::Vector2f(2400.,1500.));
    background.setPosition(-1200.,-750.);
    background.setFillColor(sf::Color(240,240,240));
    sim_zone.setFillColor(sf::Color(50,150,255));
    sf::View view;
    view.reset(sf::FloatRect(-1200., -750., 2400., 1500.));
    view.setViewport(sf::FloatRect(0.f, 0.f, 1.f, 1.f));
    window.setView(view);

    std::vector<Boid> boids;
    std::vector<sf::CircleShape> boid_triangles;
    int n_boids=20; //number of boids, to enter in input
    if(n_boids<=0) throw E_InvalidNumberOfBoids{};
    double close_radius = 50.;
    double sep_radius = 100.; 
    double sep_factor = 5.; 
    double align_factor = 0.9;
    double cohes_factor = 0.05;

    double sim_radius = std::sqrt(MAX_RADIUS2);
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    for(int i=0; i<n_boids; ++i) {
      double spawn_radius = sim_radius * std::rand()/(RAND_MAX+5E9); //change names, these are polar coords
      Angle spawn_angle{360. * std::rand()/RAND_MAX};
      Position spawn_pos{spawn_radius * spawn_angle.getCosine(), spawn_radius * spawn_angle.getSine()};
      double spawn_speed = std::sqrt(MAX_SPEED2 - 5E13) * std::rand()/(RAND_MAX+1E8); //change names, these are polar coords
      Angle spawnspeed_angle{360. * std::rand()/RAND_MAX};
      Velocity spawn_vel{spawn_speed * spawn_angle.getCosine(), spawn_radius * spawn_angle.getSine()};
      boids.push_back(Boid(spawn_pos,spawn_vel));

      boid_triangles.push_back(sf::CircleShape(12.,3)); //a triangle is just a circle approxed with 3 points
      boid_triangles[i].setFillColor(sf::Color::Black);
      boid_triangles[i].setPosition(boids[i].getPosition().getX(), boids[i].getPosition().getY());
      boid_triangles[i].setRotation(- boids[i].getAngle().getDegrees());
      window.draw(boid_triangles[i]);
    }
    assert(boids.size() == static_cast<unsigned long>(n_boids));

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
        future_boids[i].updateVelocity(boids, close_radius, sep_radius, sep_factor, align_factor, cohes_factor);
        future_boids[i].moveBoid(TIME_STEP);
      }
      boids = future_boids;
      std::this_thread::sleep_for(std::chrono::milliseconds(16));

      window.clear();
      window.draw(background);
      window.draw(sim_zone);
      for(int i = 0; i < n_boids; ++i){
        boid_triangles[i].setPosition(boids[i].getPosition().getX(), boids[i].getPosition().getY());
        boid_triangles[i].setRotation(- boids[i].getAngle().getDegrees()); //Angle goes counterclockwise, SFML goes clockwise
        window.draw(boid_triangles[i]);
      }
      window.display();

}
return 0;
}