#include "main.hpp"
#include "graphics.hpp"

int main()
{
    sf::RenderWindow window(sf::VideoMode(2400, 1500), "SFML works!");
    sf::CircleShape sim_zone(750., 100);
    sf::RectangleShape background(sf::Vector2f(2400.,1500.));
    background.setFillColor(sf::Color(240,240,240));
    sim_zone.setFillColor(sf::Color(50,150,255));

    std::vector<Boid> boids;
    std::vector<sf::ConvexShape> boid_triangles;
    int n_boids=10; //number of boids, to enter in input
    if(n_boids<=0) throw E_InvalidNumberOfBoids{};
    double close_radius = 100.;
    double sep_radius = 1; 
    double sep_factor = 100.; 
    double align_factor = 0.5;
    double cohes_factor = 0.5;

    double sim_radius = std::sqrt(MAX_RADIUS2);
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    for(int i=0; i<n_boids; ++i) {
      double spawn_radius = sim_radius * std::rand()/RAND_MAX; //change names, these are polar coords
      Angle spawn_angle{360. * std::rand()/RAND_MAX};
      Position spawn_pos{spawn_radius * spawn_angle.getCosine(), spawn_radius * spawn_angle.getSine()};
      boids.push_back(Boid(spawn_pos));

      boid_triangles.push_back(sf::ConvexShape(3));
      boid_triangles[i].setPoint(1, sf::Vector2f{0.,10.});
      boid_triangles[i].setPoint(2, sf::Vector2f{3.,0});
      boid_triangles[i].setPoint(3, sf::Vector2f{-3.,0.});
      boid_triangles[i].setPosition(boids[i].getPosition().getX(), boids[i].getPosition().getY());
      boid_triangles[i].setRotation(boids[i].getAngle().getDegrees());
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

      window.clear();
      window.draw(background);
      window.draw(sim_zone);
      for(int i = 0; i < n_boids; ++i){
        boid_triangles[i].setPosition(boids[i].getPosition().getX(), boids[i].getPosition().getY());
        boid_triangles[i].setRotation(boids[i].getAngle().getDegrees());
        window.draw(boid_triangles[i]);
      }
      window.display();

}
return 0;
}