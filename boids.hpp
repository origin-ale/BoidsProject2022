#ifndef BOIDS_HPP
#define BOIDS_HPP

//----------CONSTANTS AND EXCEPTIONS----------

constexpr double pi = 3.141592653589793238462643383279; //numbers library is C++20 onwards

constexpr double MAX_RADIUS2 = 25E6;  // maximum value of boid.getPosition.getNorm2()
struct E_OutOfBounds{}; // exception thrown for radii higher than MAX_RADIUS

constexpr double MAX_SPEED2 = 40000.;  // maximum value of boid.getVelocity.getNorm2()
struct E_SpeedTooHigh{};  // exception thrown for speeds higher than MAX_SPEED

struct E_InvalidAngle{}; //exception thrown for angles lower than 0 or higher than 360 degrees. Not thrown immediately if invalid angle is put into constructor, only if it somehow manages to not get converted into 0 to 360 degs (INFINITY, HUGE_VAL, NaN)

constexpr double TIME_STEP = 1./60.; //default time step for evolution (60 fps)
struct E_InvalidMovementTime{}; //exception thrown for overly long times passed to moveBoid

//----------CLASS DEFINITIONS AND MEMBER DECLARATIONS----------

class Position { //class to handle object positions
  double x=0.; //x coord of object (0 is center)
  double y=0.; //y coord of object (0 is center)

  public:
  explicit Position(double, double);  //simple constructor, takes x and y coords 

  double getX() const;  //returns X position
  double getY() const;  //returns Y position
  double getNorm2()const; //returns velocity norm squared
};

bool operator==(Position const&, Position const&);

class Velocity { //class to handle object velocities
  double vx=0.; //x velocity of object
  double vy=0.; //y velocity of object

  public:
  explicit Velocity(double, double); //simple constructor, takes x and y velocities

  double getXVel() const; //returns X velocity
  double getYVel() const; //returns Y velocity
  double getNorm2() const; //returns velocity norm squared
};

bool operator==(Velocity const&, Velocity const&);

//maybe use named constructors to build both in degs and rads
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

class Boid {  // each boid is one of these
  private:
  Position pos; // position of boid, with x and y coords
  Velocity vel; // velocity of boid, vector with x and y components
  Angle agl; // angle boid is facing, in degrees (counterclockwise from east)

  public:
  Boid(Position const&, Velocity const&, Angle const&); //simple constructor, takes spawn position, angle and velocity
  Boid(); //no argument constructor, spawns an unrotated, still boid at (0,0)
  Boid(Position const&);  //position-only constructor, spawns an unrotated, still boid at specified position
  Boid(Position const&, Velocity const&);  //position-velocity constructor, spawns an unrotated boid at specified position with specified velocity

  Position getPosition() const; //returns position of boid
  Velocity getVelocity() const; //returns velocity of boid
  Angle getAngle() const;  //returns angle of boid

  Position moveBoid(double);  //moves boid by a step in time

  Position setPosition(Position const&);
  Velocity setVelocity(Velocity const&);
  Angle setAngle(Angle const&);
};

#endif