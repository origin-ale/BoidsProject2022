#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <cmath>
#include <vector>
#include <cassert>
#include <iostream>

//----------CONSTANTS AND EXCEPTIONS----------

constexpr double pi = 3.141592653589793238462643383279; //numbers library is C++20 onwards

constexpr double MAX_RADIUS2 =  1E6;  // maximum value of boid.getPosition().getNorm2()
struct E_OutOfBounds{}; // exception thrown for radii higher than MAX_RADIUS

constexpr double MAX_SPEED2 = 1E10;  // maximum value of boid.getVelocity().getNorm2()
struct E_SpeedTooHigh{};  // exception thrown for speeds higher than MAX_SPEED

struct E_InvalidAngle{}; //exception thrown for angles lower than 0 or higher than 360 degrees. Not thrown immediately if invalid angle is put into constructor, only if it somehow manages to not get converted into 0 to 360 degs (INFINITY, HUGE_VAL, NaN)

struct E_InvalidMovementTime{}; //exception thrown for overly long times passed to moveBoid

//----------CLASS DEFINITIONS AND MEMBER DECLARATIONS----------

class Coords {  //abstract base class for x and y coordinates
  protected: private:
  double x=0.;
  double y=0.;

  public:
  explicit Coords(double, double);

  virtual double getX() const;  //returns X position
  virtual double getY() const;  //returns Y position
  virtual double getNorm2() const; //returns norm squared
};

bool operator==(Coords const&, Coords const&);
Coords operator-(Coords const&, Coords const&);
Coords operator+(Coords const&, Coords const&);

class Position : public Coords { //derived class to handle object positions

  public:
  explicit Position(double, double);  //simple constructor, takes x and y position coords

};


class Velocity : public Coords { //derived class to handle object velocities

  public:
  explicit Velocity(double, double); //simple constructor, takes x and y velocities

};



class Angle{  //class to handle angles both in degrees (SFML) and radians (C++ trig functions)
  private:
  double angle=0.;  //angle in degrees

  public:
  explicit Angle(double); //constructor from degrees

  double getDegrees() const; //returns angle in degrees
  double getRadians() const; //returns angle in radians

  double getSine() const; //returns sine of angle
  double getCosine() const; //returns cosine of angle
};

bool operator==(Angle const&, Angle const&);
bool operator<(Angle const&, Angle const&);
bool operator>(Angle const&, Angle const&);
bool operator<=(Angle const&, Angle const&);
bool operator>=(Angle const&, Angle const&);
Angle operator+(Angle const&, Angle const&);
Angle operator-(Angle const&, Angle const&);

class Boid {  // each boid is one of these
  private:
  Position pos; // position of boid, with x and y coords
  Velocity vel; // velocity of boid, vector with x and y components
  Angle agl; // angle boid is facing, in degrees (counterclockwise from east)
  unsigned int flk; //flock boid belongs to

  public:
  Boid(Position const&, Velocity const&, Angle const&); //simple constructor, takes spawn position, angle and velocity. Spawns a boid in flock 0.
  Boid(Position const&, Velocity const&, Angle const&, unsigned int); //spawns boid with specified attributes and flock
  Boid(); //no argument constructor, spawns an unrotated, still boid at (0,0)
  Boid(Position const&);  //position-only constructor, spawns an unrotated, still boid at specified position
  Boid(Position const&, Velocity const&);  //position-velocity constructor, spawns an unrotated boid at specified position with specified velocity

  Position getPosition() const; //returns position of boid
  Velocity getVelocity() const; //returns velocity of boid
  Angle getAngle() const;  //returns angle of boid
  int getFlock() const; //returns number of boid's flock

  Position moveBoid(double);  //moves boid by a step in time
  Velocity updateBoidVelocity(std::vector<Boid> const, std::vector<Boid> const, double, double, double, double, double, Angle = Angle(180.));  //applies flight rules to ordinary boid
  Velocity updatePredatorVelocity(std::vector<Boid> const, std::vector<Boid> const, double, double, double, double, double, Angle = Angle(180.));  //applies flight rules to predator

  Position setPosition(Position const&);
  Velocity setVelocity(Velocity const&);
  Angle setAngle(Angle const&);
  int setFlock(int const&);
};
#endif
