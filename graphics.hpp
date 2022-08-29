#ifndef GRAPHICS_HPP
#define GRAPHICS_HPP

#include <SFML/Graphics.hpp>
#include <vector>
#include "boids.hpp"
#include "SelbaWard/Common.hpp"
#include "SelbaWard/Ring.hpp"

void initializeGraphic(Boid const&, sf::CircleShape const&, std::vector<sf::CircleShape>&, std::vector<sw::Ring>&, Angle, sf::Color const&, sf::Color const&, sf::RenderWindow&, double); //Add the boid graphic to a vector and render it 
void updateGraphic(Boid const&, sf::CircleShape&, sw::Ring&, sf::RenderWindow&, double);

#endif