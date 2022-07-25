#include <cassert>
#include <iostream>
#include "boids.hpp"
#include <cmath>

//-----Definitions for Position-----

Position::Position(double x_coord, double y_coord):x{x_coord}, y{y_coord} { // Position constructor
  if(getNorm2() > MAX_RADIUS2 || !(isfinite(getNorm2()))) throw E_OutOfBounds{}; // check object isn't out of bounds when spawned. If it is, raise exception.
  assert(getNorm2() <= MAX_RADIUS2); // terminate if somehow still greater
}

double Position::getX() const{
  return x;
}

double Position::getY() const{
  return y;
}

double Position::getNorm2() const{ // returns norm^2 of position
  return x * x + y * y;
}

bool operator==(Position const& lhs, Position const& rhs){
  return lhs.getX() == rhs.getX() && lhs.getY() == rhs.getY();
}

//-----Definitions for Velocity-----

Velocity::Velocity(double x_vel, double y_vel):vx{x_vel}, vy{y_vel} { // Velocity constructor
  if(getNorm2() > MAX_SPEED2 || !(isfinite(getNorm2()))) throw E_SpeedTooHigh{}; // check object isn't over speed limit when spawned. If it is, raise exception.
  assert(getNorm2() <= MAX_SPEED2); // terminate if somehow still greater
}

double Velocity::getXVel() const{
  return vx;
}

double Velocity::getYVel() const{
  return vy;
}

double Velocity::getNorm2() const{ // returns norm^2 of velocity
  return vx * vx + vy * vy;
}

bool operator==(Velocity const& lhs, Velocity const& rhs){
  return lhs.getXVel() == rhs.getXVel() && lhs.getYVel() == rhs.getYVel();
}

//-----Definitions for Angle-----

Angle::Angle(double raw_angle){ //CHECK THROWING ORDER
  double temp; //temp variable to make throw statement more readable
  if(raw_angle > 360.) raw_angle = std::remainder(raw_angle, 360.);  //if remainder method is applicable use it
  else if(raw_angle < 0.){  
    while(raw_angle < 0.){ //otherwise keep adding 360 to it until it is in the correct range
      raw_angle += 360.;
    }
  }
  temp = raw_angle; //and set temp to raw_angle

  if(temp < 0.|| temp > 360. || !(isfinite(temp))) throw E_InvalidAngle{}; 
  angle = temp; //remainder of integer division raw_angle / 360
  assert(getDegrees() >= 0. && getDegrees() <= 360.); //class invariants, in terms of getDegrees to facilitate switching internal rep to radiants
}

double Angle::getDegrees() const{
  return angle;
}

double Angle::getRadians() const{
  return (angle / 360.) * 2. * pi;
}

double Angle::getSine() const{
  return std::sin(getRadians());
}

double Angle::getCosine() const{
  return std::cos(getRadians());
}

bool operator==(Angle const& lhs, Angle const& rhs){
  return lhs.getDegrees() == rhs.getDegrees();
}

bool operator<(Angle const& lhs, Angle const& rhs){
  return lhs.getDegrees() < rhs.getDegrees();
}

bool operator>(Angle const& lhs, Angle const& rhs){ //implemented in terms of the other two, standard procedure
  return !(lhs.getDegrees() < rhs.getDegrees() || lhs.getDegrees() == rhs.getDegrees()) ;
}

std::ostream& operator<< (std::ostream& os, const Angle& angle) { //to print by just writing var name ("<< angle"), allows doctest to print wrong values
    os << angle.getDegrees();
    return os;
}

//-----Definitions for Boid-----

Boid::Boid(Position const& spawn_pos, Velocity const& spawn_vel, Angle const& spawn_agl): pos{spawn_pos}, vel{spawn_vel}, agl{spawn_agl} { //basic constructor
  assert(pos.getNorm2() <= MAX_RADIUS2);  //asserts to check invariants
  assert(agl.getDegrees() <= 360.);
  assert(agl.getDegrees() >= 0.);
  assert(vel.getNorm2() <= MAX_SPEED2);
}

Boid::Boid(): pos{Position(0.,0.)}, vel{Velocity(0.,0.)}, agl{Angle(0.)} {} //no-argument constructor

Boid::Boid(Position const& spawn_pos): pos{spawn_pos}, vel{Velocity(0.,0.)}, agl{Angle(0.)} //position-only constructor
{
  assert(pos.getNorm2() <= MAX_RADIUS2);  //no need for other asserts, initialization values are known
}

Boid::Boid(Position const& spawn_pos, Velocity const& spawn_vel): pos{spawn_pos}, vel{spawn_vel}, agl{Angle(0.)} { //position-velocity constructor
  assert(pos.getNorm2() <= MAX_RADIUS2); 
  assert(vel.getNorm2() <= MAX_SPEED2);
}

Position Boid::getPosition() const{
  return pos;
}

Velocity Boid::getVelocity() const{
  return vel;
}

Angle Boid::getAngle() const{
  return agl;
}

Position Boid::moveBoid(double delta_t){
  if(!(isfinite(delta_t))) throw E_InvalidMovementTime{};
  double future_x = pos.getX() + vel.getXVel() * delta_t;
  double future_y = pos.getY() + vel.getYVel() * delta_t;
  //only move boid if both future coords are in bounds (TEMP, maybe implement polar coordinates)
  if(!(isfinite(future_x)) || !(isfinite(future_y)) || future_x * future_x + future_y * future_y > MAX_RADIUS2) setPosition(Position(pos.getX(), pos.getY()));
  else setPosition(Position(future_x, future_y));
  assert(pos.getX() * pos.getX() + pos.getY() * pos.getY() <= MAX_RADIUS2);
  return pos;
}

//ADD CATCHES TO SET FUNCTIONS

Position Boid::setPosition(Position const& newpos){
  pos = newpos;
  return pos;
}

Velocity Boid::setVelocity(Velocity const& newvel){
  vel = newvel;
  return vel;
}

Angle Boid::setAngle(Angle const& newagl){
  agl = newagl;
  return agl;
}