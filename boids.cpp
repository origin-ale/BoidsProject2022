#include "boids.hpp"

Coords::Coords(double x_coord, double y_coord)
    : x{x_coord}, y{y_coord} {}  // abstract base class

double Coords::getX() const { return x; }

double Coords::getY() const { return y; }

double Coords::getNorm2() const { return x * x + y * y; }

bool operator==(Coords const& lhs, Coords const& rhs) {
  return lhs.getX() == rhs.getX() && lhs.getY() == rhs.getY();
}

Coords operator+=(Coords& lhs, Coords const& rhs) {
  return lhs = Coords(lhs.getX() + rhs.getX(), lhs.getY() + rhs.getY());
}

Coords operator+(Coords const& lhs, Coords const& rhs) {
  Coords temp = lhs;
  return temp += rhs;
}

Coords operator-(Coords const& lhs, Coords const& rhs) {
  return Position(lhs.getX() - rhs.getX(), lhs.getY() - rhs.getY());
}

//-----Definitions for Position-----

Position::Position(double x_, double y_)
    : Coords{x_, y_} {}  // Position constructor

//-----Definitions for Velocity-----

Velocity::Velocity(double x_coord, double y_coord)
    : Coords{x_coord, y_coord} {  // Velocity constructor
  if (getNorm2() > MAX_SPEED2 || !(std::isfinite(getNorm2())))
    throw E_SpeedTooHigh{};          // check object isn't over speed limit when
                                     // spawned. If it is, raise exception.
  assert(getNorm2() <= MAX_SPEED2);  // terminate if somehow still greater
}

//-----Definitions for Angle-----

Angle::Angle(double raw_angle) {
  double temp;  // temp variable to make throw statement more readable
  if (raw_angle > 360.)
    raw_angle = std::remainder(
        raw_angle, 360.);  // if remainder method is applicable use it
  else if (raw_angle < 0.) {
    while (raw_angle < 0.) {  // otherwise keep adding 360 to it until it is in
                              // the correct range
      raw_angle += 360.;
    }
  }
  temp = raw_angle;  // and set temp to raw_angle

  if (temp < 0. || temp > 360. || !(std::isfinite(temp)))
    throw E_InvalidAngle{};
  angle = temp;  // remainder of integer division raw_angle / 360
  assert(getDegrees() >= 0. &&
         getDegrees() <=
             360.);  // class invariants, in terms of getDegrees to facilitate
                     // switching internal rep to radiants
}

double Angle::getDegrees() const { return angle; }

double Angle::getRadians() const { return (angle / 360.) * 2. * pi; }

double Angle::getSine() const { return std::sin(getRadians()); }

double Angle::getCosine() const { return std::cos(getRadians()); }

bool operator==(Angle const& lhs, Angle const& rhs) {
  return lhs.getDegrees() == rhs.getDegrees();
}

bool operator<(Angle const& lhs, Angle const& rhs) {
  return lhs.getDegrees() < rhs.getDegrees();
}

bool operator>(Angle const& lhs,
               Angle const& rhs) {  // implemented in terms of the other two,
                                    // standard procedure
  return !(lhs.getDegrees() < rhs.getDegrees() ||
           lhs.getDegrees() == rhs.getDegrees());
}

bool operator<=(Angle const& lhs, Angle const& rhs) {
  return (lhs < rhs || lhs == rhs);
}

bool operator>=(Angle const& lhs, Angle const& rhs) {
  return (lhs > rhs || lhs == rhs);
}

Angle operator+(Angle const& lhs, Angle const& rhs) {
  return Angle(lhs.getDegrees() + rhs.getDegrees());
}

Angle operator-(Angle const& lhs, Angle const& rhs) {
  return Angle(lhs.getDegrees() + rhs.getDegrees());
}

std::ostream& operator<<(
    std::ostream& os,
    const Angle& angle) {  // to print by just writing var name ("<< angle"),
                           // allows doctest to print wrong values
  os << angle.getDegrees();
  return os;
}

//-----Definitions for Boid-----

Boid::Boid(Position const& spawn_pos, Velocity const& spawn_vel,
           Angle const& spawn_agl)
    : pos{spawn_pos}, vel{spawn_vel}, agl{spawn_agl}, flk{0} {  // basic
                                                                // constructor
  if (pos.getNorm2() > MAX_RADIUS2 || !(std::isfinite(pos.getNorm2())))
    throw E_OutOfBounds{};  // check object isn't out of bounds when spawned. If
                            // it is, raise exception.
  assert(pos.getNorm2() <= MAX_RADIUS2);  // asserts to check invariants
  assert(agl.getDegrees() <= 360.);
  assert(agl.getDegrees() >= 0.);
  assert(vel.getNorm2() <= MAX_SPEED2);
}

Boid::Boid(Position const& spawn_pos, Velocity const& spawn_vel,
           Angle const& spawn_agl, unsigned int flock)
    : pos{spawn_pos},
      vel{spawn_vel},
      agl{spawn_agl},
      flk{flock} {  // flock constructor
  if (pos.getNorm2() > MAX_RADIUS2 || !(std::isfinite(pos.getNorm2())))
    throw E_OutOfBounds{};  // check object isn't out of bounds when spawned. If
                            // it is, raise exception.
  assert(pos.getNorm2() <= MAX_RADIUS2);  // asserts to check invariants
  assert(agl.getDegrees() <= 360.);
  assert(agl.getDegrees() >= 0.);
  assert(vel.getNorm2() <= MAX_SPEED2);
}

Boid::Boid()
    : pos{Position(0., 0.)},
      vel{Velocity(0., 0.)},
      agl{Angle(0.)},
      flk{0} {}  // no-argument constructor

Boid::Boid(Position const& spawn_pos)
    : pos{spawn_pos},
      vel{Velocity(0., 0.)},
      agl{Angle(0.)},
      flk{0}  // position-only constructor
{
  assert(pos.getNorm2() <= MAX_RADIUS2);  // no need for other asserts,
                                          // initialization values are known
}

Boid::Boid(Position const& spawn_pos, Velocity const& spawn_vel)
    : pos{spawn_pos}, vel{spawn_vel}, agl{Angle(0.)}, flk{0} {  // position-velocity
                                                                // constructor
  assert(pos.getNorm2() <= MAX_RADIUS2);
  assert(vel.getNorm2() <= MAX_SPEED2);
}

Position Boid::getPosition() const { return pos; }

Velocity Boid::getVelocity() const { return vel; }

Angle Boid::getAngle() const { return agl; }

int Boid::getFlock() const { return flk; }

Position Boid::setPosition(Position const& newpos) {
  pos = newpos;
  if (pos.getNorm2() > MAX_RADIUS2 || !(std::isfinite(pos.getNorm2())))
    throw E_OutOfBounds{};
  return pos;
}

Velocity Boid::setVelocity(Velocity const& newvel) {
  vel = newvel;
  return vel;
}

Angle Boid::setAngle(Angle const& newagl) {
  agl = newagl;
  return agl;
}

int Boid::setFlock(int const& newflk) {
  flk = newflk;
  return flk;
}

Position Boid::moveBoid(double delta_t) {
  if (!(std::isfinite(delta_t))) throw E_InvalidMovementTime{};

  double future_x = pos.getX() + vel.getX() * delta_t;
  double future_y = pos.getY() + vel.getY() * delta_t;
  try {
    setPosition(Position(future_x, future_y));
  } catch (E_OutOfBounds) {
    Angle pos_angle{360 * std::atan2(pos.getY(), pos.getX()) / (2 * pi)};
    setPosition(Position(pos_angle.getCosine() * std::sqrt(MAX_RADIUS2 - 1),
                         pos_angle.getSine() * std::sqrt(MAX_RADIUS2 - 1)));
    setVelocity(Velocity(-vel.getX(), -vel.getY()));
  }
  assert(pos.getNorm2() <= MAX_RADIUS2);
  return pos;
}

Velocity Boid::updateBoidVelocity(std::vector<Boid> const boids,
                                  std::vector<Boid> const predators,
                                  double close_radius, double sep_radius,
                                  double sep_factor, double align_factor,
                                  double cohes_factor, Angle sight_angle) {
  // initialization of component wise sums of members of nearby boids other than
  // self
  double nboids_nearby{0.};

  double sum_pos_boid_x{0.};
  double sum_pos_boid_y{0.};
  int nboids_in_sep{0};  // used in separation

  double sum_vel_x{0.};
  double sum_vel_y{0.};  // used in alignment

  double sum_pos_center_x{0.};
  double sum_pos_center_y{0.};  // used in cohesion

  double sum_pos_pred_x{0.};
  double sum_pos_pred_y{0.};
  int npreds_in_sep{0};  // used in predator velocity

  for (int j = 0; static_cast<unsigned long>(j) < boids.size();
       ++j) {  // this cycle calculates sums for ordinary boids

    // initialization of auxiliary variables
    Position pj{boids[j].getPosition()};
    double dij{sqrt((pos - pj).getNorm2())};
    Velocity vj{boids[j].getVelocity()};
    Angle aj{360. * std::atan2(pj.getY() - pos.getY(), pj.getX() - pos.getX()) /
             (2 * pi)};

    bool sight_condition;  // selects boids in sight range
    if (agl.getDegrees() - sight_angle.getDegrees() < 0. ||
        agl.getDegrees() + sight_angle.getDegrees() >=
            360.) {  // if 0 degrees is in the sight range
      sight_condition =
          (aj >= Angle(agl.getDegrees() - sight_angle.getDegrees()) ||
           aj <= Angle(agl.getDegrees() + sight_angle.getDegrees()));
    } else {
      sight_condition =
          (aj >= Angle(agl.getDegrees() - sight_angle.getDegrees()) &&
           aj <= Angle(agl.getDegrees() + sight_angle.getDegrees()));
    }

    if (sight_condition == true  // only apply flight rules to boids that are
                                 // within the sight range, ...
        && dij <= close_radius && dij != 0.      //... nearby excluding self ...
        && boids[j].getFlock() == getFlock()) {  //... and in same flock

      if (dij <
          sep_radius) {  // separation component due to nearby ordinary boids
        sum_pos_boid_x += pj.getX();
        sum_pos_boid_y += pj.getY();
        ++nboids_in_sep;
      }

      sum_vel_x += vj.getX();
      sum_vel_y += vj.getY();

      sum_pos_center_x += pj.getX();
      sum_pos_center_y += pj.getY();

      ++nboids_nearby;
    }
  }

  for (int k = 0; static_cast<unsigned long>(k) < predators.size();
       ++k) {  // this cycle calculates separation component due to nearby
               // predators

    Position pred_pk{predators[k].getPosition()};
    double pred_dik{sqrt((pos - pred_pk).getNorm2())};
    Angle ak{
        360. *
        std::atan2(pred_pk.getY() - pos.getY(), pred_pk.getX() - pos.getX()) /
        (2 * pi)};

    if (pred_dik <
        5 * sep_radius) {  // effect of separation from predators is greater
                           // than that of separation from other boids
      sum_pos_pred_x += pred_pk.getX();
      sum_pos_pred_y += pred_pk.getY();
      ++npreds_in_sep;
    }
  }

  // velocity for ordinary boids
  double sum_pos_sep_x = sum_pos_boid_x + sum_pos_pred_x;
  double sum_pos_sep_y = sum_pos_boid_y + sum_pos_pred_y;
  double nsep = nboids_in_sep + npreds_in_sep;

  double sep_vel_x = sep_factor * (nsep * pos.getX() - sum_pos_sep_x);
  double sep_vel_y = sep_factor * (nsep * pos.getY() - sum_pos_sep_y);
  Velocity sep_vel{sep_vel_x, sep_vel_y};

  double align_vel_x{0.};
  double align_vel_y{0.};
  double cohes_vel_x{0.};
  double cohes_vel_y{0.};
  if (nboids_nearby != 0) {  // avoid dividing by 0 if there are no close boids
    align_vel_x =
        align_factor * ((1 / (nboids_nearby)) * sum_vel_x - vel.getX());
    align_vel_y =
        align_factor * ((1 / (nboids_nearby)) * sum_vel_y - vel.getY());

    double near_centermass_x = sum_pos_center_x / (nboids_nearby);
    double near_centermass_y = sum_pos_center_y / (nboids_nearby);
    cohes_vel_x = cohes_factor * (near_centermass_x - pos.getX());
    cohes_vel_y = cohes_factor * (near_centermass_y - pos.getY());
  }
  Velocity align_vel{align_vel_x, align_vel_y};
  Velocity cohes_vel{cohes_vel_x, cohes_vel_y};

  // edge velocity
  double edge_factor = 50.;  // not in input, not supposed to be modified
  double edge_limit_sq = 1E4;
  Angle pos_angle{360. * std::atan2(pos.getY(), pos.getX()) / (2 * pi)};
  Velocity edge_vel{0., 0.};
  if (MAX_RADIUS2 - pos.getNorm2() < edge_limit_sq) {
    double edge_vel_x = -edge_factor *
                        (1. / std::sqrt(MAX_RADIUS2 - pos.getNorm2())) *
                        pos_angle.getCosine();
    double edge_vel_y = -edge_factor *
                        (1. / std::sqrt(MAX_RADIUS2 - pos.getNorm2())) *
                        pos_angle.getSine();
    try {
      edge_vel = Velocity(edge_vel_x, edge_vel_y);
    } catch (E_SpeedTooHigh) {
      edge_vel = Velocity((0.1 * MAX_SPEED2) * pos_angle.getCosine(),
                          (0.1 * MAX_SPEED2) * pos_angle.getSine());
    }
  }

  try {
    vel += (sep_vel + align_vel + cohes_vel + edge_vel);
  } catch (E_SpeedTooHigh) {
    vel = Velocity(agl.getCosine() * std::sqrt(MAX_SPEED2 - 1.),
                   agl.getSine() * std::sqrt(MAX_SPEED2 - 1.));
  }

  // update angle
  agl = Angle(360. * std::atan2(vel.getY(), vel.getX()) / (2 * pi));

  return vel;
}

Velocity Boid::updatePredatorVelocity(std::vector<Boid> const predators,
                                      std::vector<Boid> const boids,
                                      double close_radius, double sep_radius,
                                      double sep_factor, double align_factor,
                                      double cohes_factor, Angle sight_angle) {
  double sum_vel_x{0.};
  double sum_vel_y{0.};  // used in alignment with ordinary boids

  double sum_pos_center_x{0.};
  double sum_pos_center_y{0.};
  int nboids_nearby{0};  // used in cohesion with ordinary boids

  double sum_pos_pred_x{0.};
  double sum_pos_pred_y{0.};
  int npreds_in_sep{0};  // used in separation from other predators

  for (int j = 0; static_cast<unsigned long>(j) < boids.size();
       ++j) {  // this cycle calculates all sums but does not set velocities

    // initialization of auxiliary variables
    Position pj{boids[j].getPosition()};
    double dij{sqrt((pos - pj).getNorm2())};
    Velocity vj{boids[j].getVelocity()};
    Angle aj{360. * std::atan2(pj.getY() - pos.getY(), pj.getX() - pos.getX()) /
             (2 * pi)};

    bool sight_condition;  // selects boids in sight range
    if (agl.getDegrees() - sight_angle.getDegrees() < 0. ||
        agl.getDegrees() + sight_angle.getDegrees() >=
            360.) {  // if 0 degrees is in the sight range
      sight_condition =
          (aj >= Angle(agl.getDegrees() - sight_angle.getDegrees()) ||
           aj <= Angle(agl.getDegrees() + sight_angle.getDegrees()));
    } else {
      sight_condition =
          (aj >= Angle(agl.getDegrees() - sight_angle.getDegrees()) &&
           aj <= Angle(agl.getDegrees() + sight_angle.getDegrees()));
    }

    if (sight_condition == true &&
        dij <= close_radius) {  // only apply flight rules to close boids

      sum_pos_center_x += pj.getX();
      sum_pos_center_y += pj.getY();

      sum_vel_x += vj.getX();
      sum_vel_y += vj.getY();

      ++nboids_nearby;
    }
  }

  for (int k = 0; static_cast<unsigned long>(k) < predators.size();
       ++k) {  // this cycle calculates separation component due to nearby
               // predators

    Position pred_pk{predators[k].getPosition()};
    double pred_dik{sqrt((pos - pred_pk).getNorm2())};
    Angle pred_ak{
        360. *
        std::atan2(pred_pk.getY() - pos.getY(), pred_pk.getX() - pos.getX()) /
        (2 * pi)};

    bool sight_condition;  // selects predators in sight range
    if (agl.getDegrees() - sight_angle.getDegrees() < 0. ||
        agl.getDegrees() + sight_angle.getDegrees() >=
            360.) {  // if 0 degrees is in the sight range
      sight_condition =
          (pred_ak >= Angle(agl.getDegrees() - sight_angle.getDegrees()) ||
           pred_ak <= Angle(agl.getDegrees() + sight_angle.getDegrees()));
    } else {
      sight_condition =
          pred_ak >= Angle(agl.getDegrees() - sight_angle.getDegrees()) &&
          pred_ak <= Angle(agl.getDegrees() + sight_angle.getDegrees());
    }

    if (sight_condition == true && pred_dik < sep_radius) {
      sum_pos_pred_x += pred_pk.getX();
      sum_pos_pred_y += pred_pk.getY();
      ++npreds_in_sep;
    }
  }

  // separation velocity due to other predators only
  double sep_vel_x = sep_factor * (npreds_in_sep * pos.getX() - sum_pos_pred_x);
  double sep_vel_y = sep_factor * (npreds_in_sep * pos.getY() - sum_pos_pred_y);
  Velocity sep_vel{sep_vel_x, sep_vel_y};

  // cohesion velocity in order for the predator to chase ordinary boids
  double align_vel_x{0.};
  double align_vel_y{0.};
  double cohes_vel_x{0.};
  double cohes_vel_y{0.};
  if (nboids_nearby != 0) {  // avoid dividing by 0 if there are no close boids
    align_vel_x =
        align_factor * ((1 / (nboids_nearby)) * sum_vel_x - vel.getX());
    align_vel_y =
        align_factor * ((1 / (nboids_nearby)) * sum_vel_y - vel.getY());

    double near_centermass_x = sum_pos_center_x / (nboids_nearby);
    double near_centermass_y = sum_pos_center_y / (nboids_nearby);
    cohes_vel_x = cohes_factor * (near_centermass_x - pos.getX());
    cohes_vel_y = cohes_factor * (near_centermass_y - pos.getY());
  }
  Velocity align_vel{align_vel_x, align_vel_y};
  Velocity cohes_vel{cohes_vel_x, cohes_vel_y};

  // edge velocity
  double edge_factor = 5.;  // not in input, not supposed to be modified
  double edge_limit_sq = 1E4;
  Angle pos_angle{360 * std::atan2(pos.getY(), pos.getX()) / (2 * pi)};
  Velocity edge_vel{0., 0.};
  if (MAX_RADIUS2 - pos.getNorm2() < edge_limit_sq) {
    double edge_vel_x = -edge_factor *
                        (1. / std::sqrt(MAX_RADIUS2 - pos.getNorm2())) *
                        pos_angle.getCosine();
    double edge_vel_y = -edge_factor *
                        (1. / std::sqrt(MAX_RADIUS2 - pos.getNorm2())) *
                        pos_angle.getSine();
    try {
      edge_vel = Velocity(edge_vel_x, edge_vel_y);
    } catch (E_SpeedTooHigh) {
      edge_vel = Velocity((0.1 * MAX_SPEED2) * pos_angle.getCosine(),
                          (0.1 * MAX_SPEED2) * pos_angle.getSine());
    }
  }

  try {
    vel += (sep_vel + align_vel + cohes_vel + edge_vel);
  } catch (E_SpeedTooHigh) {
    vel = Velocity(agl.getCosine() * std::sqrt(MAX_SPEED2 - 1.),
                   agl.getSine() * std::sqrt(MAX_SPEED2 - 1.));
  }

  // update angle
  agl = Angle(360. * std::atan2(vel.getY(), vel.getX()) / (2 * pi));

  return vel;
}