#ifndef GRAPHICS_HPP
#define GRAPHICS_HPP

#include <SFML/Graphics.hpp>
#include <vector>
#include "boids.hpp"

void initializeGraphic(Boid const&, sf::CircleShape const&, std::vector<sf::CircleShape>&, sf::Color const&, sf::RenderWindow&, double); //Add the boid graphic to a vector and render it 
void updateGraphic(Boid const&, sf::CircleShape&, sf::RenderWindow&, double);

#endif