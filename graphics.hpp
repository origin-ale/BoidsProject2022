#ifndef GRAPHICS_HPP
#define GRAPHICS_HPP

#include <SFML/Graphics.hpp>
#include <cmath>
#include <vector>

#include "SelbaWard/Common.hpp"
#include "SelbaWard/Ring.hpp"
#include "boids.hpp"

void initializeGraphic(
    Boid const&, sf::CircleShape const&, std::vector<sf::CircleShape>&,
    std::vector<sw::Ring>&, Angle const&, sf::Color const&, sf::Color const&,
    sf::RenderWindow&,
    double);  // Add the boid graphic to a vector and render it
void updateGraphic(Boid const&, sf::CircleShape&, sw::Ring&, sf::RenderWindow&,
                   double);

#endif