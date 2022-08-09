#include <cassert>
#include <iostream>
#include "boids.hpp"
#include <cmath>
#include <vector>

//-----Definitions for Position-----

Position::Position(double x_coord, double y_coord):x{x_coord}, y{y_coord} { // Position constructor
  if(getNorm2() > MAX_RADIUS2 || !(std::isfinite(getNorm2()))) throw E_OutOfBounds{}; // check object isn't out of bounds when spawned. If it is, raise exception.
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

double operator-(Position const& lhs, Position const& rhs){
  return sqrt(lhs.getX()*lhs.getX() + lhs.getY()*lhs.getY()) - sqrt(rhs.getX()*rhs.getX() + rhs.getY()*rhs.getY());
}

//-----Definitions for Velocity-----

Velocity::Velocity(double x_vel, double y_vel):vx{x_vel}, vy{y_vel} { // Velocity constructor
  if(getNorm2() > MAX_SPEED2 || !(std::isfinite(getNorm2()))) throw E_SpeedTooHigh{}; // check object isn't over speed limit when spawned. If it is, raise exception.
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

Velocity operator+=(Velocity& lhs, Velocity const& rhs){              
    return lhs = Velocity(lhs.getXVel() + rhs.getXVel(), lhs.getYVel() + rhs.getYVel());
}

Velocity operator+(Velocity const& lhs, Velocity const& rhs){ 
    Velocity temp = lhs;             
    return temp += rhs;
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

  if(temp < 0.|| temp > 360. || !(std::isfinite(temp))) throw E_InvalidAngle{}; 
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


Position Boid::moveBoid(double delta_t){
  if(!(std::isfinite(delta_t))) throw E_InvalidMovementTime{};

  double future_x = pos.getX() + vel.getXVel() * delta_t;
  double future_y = pos.getY() + vel.getYVel() * delta_t;
  //only move boid if both future coords are in bounds (TEMP, maybe implement polar coordinates)
  if(!(std::isfinite(future_x)) || 
     !(std::isfinite(future_y)) || 
      future_x * future_x + future_y * future_y > MAX_RADIUS2
    ) setPosition(Position(pos.getX(), pos.getY())); //implement "bouncing" mechanic (poi ti spiego)
  else setPosition(Position(future_x, future_y));
  assert(pos.getX() * pos.getX() + pos.getY() * pos.getY() <= MAX_RADIUS2);
  return pos;
}


Velocity Boid::updateVelocity(std::vector<Boid> const boids, double close_radius, double sep_radius, double sep_factor, double align_factor, double cohes_factor){
  
  if (align_factor<0 || align_factor>=1) throw E_InvalidAlignmentFactor{};

  //initialization of component wise sums of members of nearby boids OTHER THAN SELF
  double sum_pos_sep_x{0.}; 
  double sum_pos_sep_y{0.}; 
  int nboids_in_sep; //used in separation

  double sum_vel_x{0.};
  double sum_vel_y{0.}; //used in alignment

  double sum_pos_center_x{0.};
  double sum_pos_center_y{0.}; //used in cohesion

  for (int j = 1; j <= boids.size(); ++j) { //this cycle calculates all sums but does not set velocities
    
    //initialization of auxiliary variables
    Position pj{boids[j].getPosition()};
    double dij{pos - pj}; //maybe redo this, not very clear
    Velocity vj{boids[j].getVelocity()};

    if(dij <= close_radius && dij != 0) {    //only apply flight rules to close boids, excluding self

      if (dij < sep_radius) {    //only add separation components if the distance between two boids is < sep_radius
        sum_pos_sep_x += pj.getX();
        sum_pos_sep_y += pj.getY();
        ++nboids_in_sep;
      }

      sum_vel_x += vj.getXVel();
      sum_vel_y += vj.getYVel(); 

      sum_pos_center_x += pj.getX();
      sum_pos_center_y += pj.getY();

      } 
  }

//calculate velocity components and add (IMPLEMENT WITH DOUBLE*POSITION AND DOUBLE*VELOCITY OPERATORS)
double sep_vel_x = - sep_factor * (nboids_in_sep * pos.getX() - sum_pos_sep_x);
double sep_vel_y = - sep_factor * (nboids_in_sep * pos.getY() - sum_pos_sep_y);
Velocity sep_vel{sep_vel_x, sep_vel_y};

double align_vel_x = align_factor * ((1/(boids.size()-1)) * sum_vel_x - vel.getXVel());
double align_vel_y = align_factor * ((1/(boids.size()-1)) * sum_vel_y - vel.getYVel());
Velocity align_vel{align_vel_x,align_vel_y};

double near_centermass_x = sum_pos_center_x / (boids.size()-1);
double near_centermass_y = sum_pos_center_y / (boids.size()-1);
double cohes_vel_x = cohes_factor * (near_centermass_x - pos.getX());
double cohes_vel_y = cohes_factor * (near_centermass_y - pos.getY());
Velocity cohes_vel{cohes_vel_x, cohes_vel_y};

vel += (sep_vel + align_vel + cohes_vel);

return vel;
}

      /* OLD SEPARATION RULE
      double separation_distance;   //separation distance, da inizializzare a boh
      for (int j = 1; j <= n_boids; ++j) {
        Position pj(boids[j].getPosition());
        double dij{ sqrt(pos.getX()*pos.getX() + pos.getY()*pos.getY()) - sqrt(pj.getX()*pj.getX() + pj.getY()*pj.getY()) };
        if(dij < separation_distance) {    //only add separation component if the distance between two boids is < sep_radius
      
        double s;  //separation factor, to enter in input
        double sumx;
        for (int j = 1; j < boids.size(); ++j) {
            sumx += ( pos.getX() - boids[j].getPosition().getX() );    
        }
        double v1x = -s * sumx;
        double sumy;
        for (int j = 1; j < boids.size(); ++j) {
            sumy += ( pos.getY() - boids[j].getPosition().getY() );  
        }
        double v1y = -s * sumy;
        setVelocity(vel + Velocity(v1x, v1y));     //added separation component v1
        }
      } */