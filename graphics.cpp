#include "graphics.hpp"

void initializeGraphic(Boid const& boid, sf::CircleShape const& shape,
                       std::vector<sf::CircleShape>& boid_triangles,
                       std::vector<sw::Ring>& sight_arcs,
                       Angle const& sight_angle, sf::Color const& boid_color,
                       sf::Color const& arc_color,
                       sf::RenderWindow& target_window, double scale) {
  boid_triangles.push_back(
      shape);  // a triangle is just a circle approxed with 3 points
  boid_triangles.back().setOrigin(shape.getRadius(), shape.getRadius());
  boid_triangles.back().setFillColor(boid_color);
  boid_triangles.back().setPosition(
      scale * boid.getPosition().getX(),
      -scale * boid.getPosition().getY());  // Y axis points downwards in SFML
  boid_triangles.back().setRotation(90. - boid.getAngle().getDegrees());

  sight_arcs.push_back(sw::Ring(2. * boid_triangles.back().getRadius(), 0.));
  sight_arcs.back().setOrigin(sight_arcs.back().getRadius(),
                              sight_arcs.back().getRadius());
  sight_arcs.back().setColor(arc_color);
  sight_arcs.back().setSectorOffset(-sight_angle.getDegrees() / 360.);
  sight_arcs.back().setSectorSize(sight_angle.getDegrees() / 180.);
  sight_arcs.back().setPosition(scale * boid.getPosition().getX(),
                                -scale * boid.getPosition().getY());
  sight_arcs.back().setRotation(90. - boid.getAngle().getDegrees());

  target_window.draw(sight_arcs.back());
  target_window.draw(boid_triangles.back());
}

void updateGraphic(Boid const& boid, sf::CircleShape& graphic,
                   sw::Ring& sight_arc, sf::RenderWindow& target_window,
                   double scale) {
  graphic.setPosition(
      scale * boid.getPosition().getX(),
      -scale * boid.getPosition().getY());  // Y axis points downwards in SFML
  graphic.setRotation(
      90 -
      boid.getAngle().getDegrees());  // Angle goes counterclockwise from right,
                                      // SFML goes clockwise from top

  sight_arc.setPosition(scale * boid.getPosition().getX(),
                        -scale * boid.getPosition().getY());
  sight_arc.setRotation(90 - boid.getAngle().getDegrees());

  target_window.draw(sight_arc);
  target_window.draw(graphic);
}