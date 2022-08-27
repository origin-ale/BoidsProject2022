#include "graphics.hpp"

void initializeGraphic(Boid const& boid, sf::CircleShape const& shape, std::vector<sf::CircleShape>& boid_triangles, sf::Color const& boid_color, sf::RenderWindow& target_window, double scale){
    boid_triangles.push_back(shape); //a triangle is just a circle approxed with 3 points
    boid_triangles.back().setOrigin(shape.getRadius()/2, shape.getRadius()/2);
    boid_triangles.back().setFillColor(boid_color);
    boid_triangles.back().setPosition(scale * boid.getPosition().getX(), - scale * boid.getPosition().getY()); //Y axis points downwards in SFML
    boid_triangles.back().setRotation(90 - boid.getAngle().getDegrees());
    target_window.draw(boid_triangles.back());
}

void updateGraphic(Boid const& boid, sf::CircleShape& graphic, sf::RenderWindow& window, double scale){
    graphic.setPosition(scale * boid.getPosition().getX(), - scale * boid.getPosition().getY()); //Y axis points downwards in SFML
    graphic.setRotation(90 - boid.getAngle().getDegrees()); //Angle goes counterclockwise from right, SFML goes clockwise from top
    window.draw(graphic);
}