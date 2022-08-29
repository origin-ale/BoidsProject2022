#include "boids.hpp"
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include<iostream>
#include<cmath>

//WRITE A BUNCH MORE TESTS

TEST_CASE("Testing position and velocity gets"){ //check a bunch of positions and velocities have correct get functions (getX, getY, getXVel, getYVel, getNorm2)
  Position pos1 = Position(2.,0.);  //second component = 0
  Position pos2 = Position(0.,5.);  //first component = 0
  Position pos3 = Position(0.,0.);  //both components = 0
  Position pos4 = Position(3.,4.);  //pitagorean triple
  Position pos5 = Position(8.,8.);  //whole comoponents
  Position pos6 = Position(102.56,568.29);  //fractional components

  Velocity vel1 = Velocity(3.,0.);  //second component = 0
  Velocity vel2 = Velocity(0.,7.);  //first component = 0
  Velocity vel3 = Velocity(0.,0.);  //both components = 0
  Velocity vel4 = Velocity(9.,12.);  //pitagorean triple
  Velocity vel5 = Velocity(4.,4.);  //whole comoponents
  Velocity vel6 = Velocity(34.56,25.89);  //fractional components

  SUBCASE("Testing component gets"){ //all but getNorm2
    CHECK(pos1.getX() == 2.);
    CHECK(pos1.getY() == 0.);
    CHECK(pos6.getX() == 102.56);
    CHECK(pos6.getY() == 568.29);

    CHECK(vel1.getXVel() == 3.);
    CHECK(vel1.getYVel() == 0.);
    CHECK(vel6.getXVel() == 34.56);
    CHECK(vel6.getYVel() == 25.89);
  }

  SUBCASE("Testing getNorm2"){

    CHECK(pos1.getNorm2() == 4.);
    CHECK(pos2.getNorm2() == 25.);
    CHECK(pos3.getNorm2() == 0.);
    CHECK(pos4.getNorm2() == 25.);
    CHECK(pos5.getNorm2() == 128.);
    CHECK(pos6.getNorm2() == doctest::Approx(333472.0777));

    CHECK(vel1.getNorm2() == 9.);
    CHECK(vel2.getNorm2() == 49.);
    CHECK(vel3.getNorm2() == 0.);
    CHECK(vel4.getNorm2() == 225.);
    CHECK(vel5.getNorm2() == 32.);
    CHECK(vel6.getNorm2() == doctest::Approx(1864.6857));
  }
}

TEST_CASE("Testing boids"){ //test boid constructors and gets work correctly
  Boid noargs_boid = Boid();  // boid with no args passed to constructor
  Boid onearg_boid = Boid(Position(1.,1.)); // boid with only position passed to constructor
  Boid twoargs_boid = Boid(Position(2.,2.),Velocity(3.,3.));
  Boid allargs_boid = Boid(Position(1.,1.), Velocity(1.,1.), Angle(90.)); //boid with all arguments passed to constructor

  // check boids have expected members
  CHECK(noargs_boid.getPosition() == Position(0.,0.));
  CHECK(noargs_boid.getVelocity() == Velocity(0.,0.));
  CHECK(noargs_boid.getAngle() == Angle(0.));


  CHECK(onearg_boid.getPosition() == Position(1.,1.));
  CHECK(onearg_boid.getVelocity() == Velocity(0.,0.));
  CHECK(onearg_boid.getAngle() == Angle(0.));

  CHECK(twoargs_boid.getPosition() == Position(2.,2.));
  CHECK(twoargs_boid.getVelocity() == Velocity(3.,3.));
  CHECK(twoargs_boid.getAngle() == Angle(0.));
  

  CHECK(allargs_boid.getPosition() == Position(1.,1.));
  CHECK(allargs_boid.getVelocity() == Velocity(1.,1.));
  CHECK(allargs_boid.getAngle() == Angle(90.));
}

TEST_CASE("Testing bounds"){  // try getting boid out of bounds
//all of these CHECK_THROWS might not work, the exceptions get caught by the constructors. In that case, change them to check for the sanitized values
  SUBCASE("Finite OoB"){
    CHECK_THROWS_AS(Boid(Position(MAX_RADIUS2,MAX_RADIUS2), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
  }

  SUBCASE("Overflowing OoB"){
    CHECK_THROWS_AS(Boid(Position(HUGE_VAL,0.), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
    CHECK_THROWS_AS(Boid(Position(0.,HUGE_VAL), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
    CHECK_THROWS_AS(Boid(Position(HUGE_VAL,HUGE_VAL), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
  }

  SUBCASE("Infinite OoB"){ //might not work
    CHECK_THROWS_AS(Boid(Position(INFINITY,0.), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
    CHECK_THROWS_AS(Boid(Position(0.,INFINITY), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
    CHECK_THROWS_AS(Boid(Position(INFINITY,INFINITY), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
  } 

  SUBCASE("NaN OoB"){ //probably won't work
    CHECK_THROWS_AS(Boid(Position(NAN,0.), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
    CHECK_THROWS_AS(Boid(Position(0.,NAN), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
    CHECK_THROWS_AS(Boid(Position(NAN,NAN), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
  } 

  SUBCASE("Mixed OoB"){
    CHECK_THROWS_AS(Boid(Position(INFINITY,HUGE_VAL), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
    CHECK_THROWS_AS(Boid(Position(HUGE_VAL,INFINITY), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
    CHECK_THROWS_AS(Boid(Position(INFINITY,NAN), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
    CHECK_THROWS_AS(Boid(Position(NAN,INFINITY), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
    CHECK_THROWS_AS(Boid(Position(HUGE_VAL,NAN), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
    CHECK_THROWS_AS(Boid(Position(NAN,HUGE_VAL), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
  } 
}

TEST_CASE("Testing speed limit"){ // try getting boid to speed higher than MAX_SPEED

  SUBCASE("Finite speed break"){
    CHECK_THROWS_AS(Boid(Position(0.,0.), Velocity(MAX_SPEED2,MAX_SPEED2), Angle(0.)), E_SpeedTooHigh);
  }

  SUBCASE("Overflowing speed break"){
    CHECK_THROWS_AS(Boid(Position(0.,0.), Velocity(HUGE_VAL,0.), Angle(0.)), E_SpeedTooHigh);
    CHECK_THROWS_AS(Boid(Position(0.,0.), Velocity(0.,HUGE_VAL), Angle(0.)), E_SpeedTooHigh);
    CHECK_THROWS_AS(Boid(Position(0.,0.), Velocity(HUGE_VAL,HUGE_VAL), Angle(0.)), E_SpeedTooHigh);
  }

  SUBCASE("Infinite speed break"){
    CHECK_THROWS_AS(Boid(Position(0.,0.), Velocity(INFINITY,0.), Angle(0.)), E_SpeedTooHigh);
    CHECK_THROWS_AS(Boid(Position(0.,0.), Velocity(0.,INFINITY), Angle(0.)), E_SpeedTooHigh);
    CHECK_THROWS_AS(Boid(Position(0.,0.), Velocity(INFINITY,INFINITY), Angle(0.)), E_SpeedTooHigh);
  }

  SUBCASE("NaN speed break"){
    CHECK_THROWS_AS(Boid(Position(0.,0.), Velocity(NAN,0.), Angle(0.)), E_SpeedTooHigh);
    CHECK_THROWS_AS(Boid(Position(0.,0.), Velocity(0.,NAN), Angle(0.)), E_SpeedTooHigh);
    CHECK_THROWS_AS(Boid(Position(0.,0.), Velocity(NAN,NAN), Angle(0.)), E_SpeedTooHigh);
  }
  SUBCASE("Mixed speed break"){
    CHECK_THROWS_AS(Boid(Position(0.,0.), Velocity(INFINITY,HUGE_VAL), Angle(0.)), E_SpeedTooHigh);
    CHECK_THROWS_AS(Boid(Position(0.,0.), Velocity(HUGE_VAL,INFINITY), Angle(0.)), E_SpeedTooHigh);
    CHECK_THROWS_AS(Boid(Position(0.,0.), Velocity(INFINITY,NAN), Angle(0.)), E_SpeedTooHigh);
    CHECK_THROWS_AS(Boid(Position(0.,0.), Velocity(NAN,INFINITY), Angle(0.)), E_SpeedTooHigh);
    CHECK_THROWS_AS(Boid(Position(0.,0.), Velocity(HUGE_VAL,NAN), Angle(0.)), E_SpeedTooHigh);
    CHECK_THROWS_AS(Boid(Position(0.,0.), Velocity(NAN,HUGE_VAL), Angle(0.)), E_SpeedTooHigh);
  }
}

TEST_CASE("Testing angles"){  //try getting boids to out-of-range angles and see if they are correctly handled
  SUBCASE("Testing finite angles"){ //these should be converted to angles in [0,360]
    Boid underrot_boid = Boid(Position(0.,0.), Velocity(0.,0.), Angle(-45.));
    Boid overrot_boid = Boid(Position(0.,0.), Velocity(0.,0.), Angle(450.));

    CHECK(underrot_boid.getAngle() == Angle(315.));
    CHECK(overrot_boid.getAngle() == Angle(90.));

    CHECK(underrot_boid.getAngle().getSine() == doctest::Approx(-std::sqrt(2)/2));
    CHECK(underrot_boid.getAngle().getCosine() == doctest::Approx(std::sqrt(2)/2));

    CHECK(overrot_boid.getAngle().getSine() == doctest::Approx(1.));
    CHECK(overrot_boid.getAngle().getCosine() == doctest::Approx(0.));
  }
  SUBCASE("Testing infinite angles"){ //these should throw and get set to 0
    CHECK_THROWS_AS(Boid(Position(0.,0.), Velocity(0.,0.), Angle(INFINITY)), E_InvalidAngle);
    CHECK_THROWS_AS(Boid(Position(0.,0.), Velocity(0.,0.), Angle(HUGE_VAL)), E_InvalidAngle);
    CHECK_THROWS_AS(Boid(Position(0.,0.), Velocity(0.,0.), Angle(NAN)), E_InvalidAngle);
  }
}

TEST_CASE("Testing everything going wrong at once"){ //try having multi-exception boids
  CHECK_THROWS(Boid(Position(MAX_RADIUS2,MAX_RADIUS2), Velocity(MAX_SPEED2,MAX_SPEED2), Angle(720.))); //finite case
  CHECK_THROWS(Boid(Position(INFINITY,NAN), Velocity(NAN,HUGE_VAL), Angle(INFINITY))); //infinite case
}

TEST_CASE("Testing movement"){ //tests move and set functions
  Boid movement_boid = Boid();  //spawn still boid
  SUBCASE("Intended behavior"){
    movement_boid.moveBoid(0.1);  //try to move it
    CHECK(movement_boid.getPosition() == Position(0.,0.)); //should have stayed still

    movement_boid.setVelocity(Velocity(1.,1.)); //set nonzero velocity
    CHECK(movement_boid.getVelocity() == Velocity(1.,1.));  //check it was set correctly
    movement_boid.moveBoid(0.1);  //move boid
    CHECK(movement_boid.getPosition() == Position(0.1,0.1));  //check it moved correctly

    movement_boid.setPosition(Position(10.,10.)); //set position directly
    CHECK(movement_boid.getPosition() == Position(10.,10.)); //check it hasn't moved since setting
    CHECK(movement_boid.getVelocity() == Velocity(1.,1.));  //check no side effect on velocity
    CHECK(movement_boid.getAngle() == Angle(0.)); //check no side effects on angle

    movement_boid.setAngle(Angle(75.)); //set nonzero angle
    movement_boid.moveBoid(1.); //move in longer time
    CHECK(movement_boid.getPosition() == Position(11.,11.));  //check correct movement
    CHECK(movement_boid.getVelocity() == Velocity(1.,1.));  //check no side effect on velocity
    CHECK(movement_boid.getAngle() == Angle(75.));  //check no side effects on angle
    }


  SUBCASE("Setting members to nonfinites"){
    movement_boid = Boid(); //reset boid to default
    REQUIRE(movement_boid.getPosition() == Position(0.,0.)); //check it was actually reset, if not then terminate test
    REQUIRE(movement_boid.getVelocity() == Velocity(0.,0.));
    REQUIRE(movement_boid.getAngle() == Angle(0.));

    //try to set position to nonfinites, constructors should throw
    CHECK_THROWS_AS(movement_boid.setPosition(Position(0.,INFINITY)), E_OutOfBounds);
    CHECK_THROWS_AS(movement_boid.setPosition(Position(INFINITY,0.)), E_OutOfBounds);
    CHECK_THROWS_AS(movement_boid.setPosition(Position(INFINITY,INFINITY)), E_OutOfBounds);
  
    CHECK_THROWS_AS(movement_boid.setPosition(Position(0.,HUGE_VAL)), E_OutOfBounds);
    CHECK_THROWS_AS(movement_boid.setPosition(Position(HUGE_VAL,0.)), E_OutOfBounds);
    CHECK_THROWS_AS(movement_boid.setPosition(Position(HUGE_VAL,HUGE_VAL)), E_OutOfBounds);

    CHECK_THROWS_AS(movement_boid.setPosition(Position(0.,NAN)), E_OutOfBounds);
    CHECK_THROWS_AS(movement_boid.setPosition(Position(NAN,0.)), E_OutOfBounds);
    CHECK_THROWS_AS(movement_boid.setPosition(Position(NAN,NAN)), E_OutOfBounds);

    //try to set velocity to nonfinites, constructors should throw
    CHECK_THROWS_AS(movement_boid.setVelocity(Velocity(0.,INFINITY)), E_SpeedTooHigh);
    CHECK_THROWS_AS(movement_boid.setVelocity(Velocity(INFINITY,0.)), E_SpeedTooHigh);
    CHECK_THROWS_AS(movement_boid.setVelocity(Velocity(INFINITY,INFINITY)), E_SpeedTooHigh);
  
    CHECK_THROWS_AS(movement_boid.setVelocity(Velocity(0.,HUGE_VAL)), E_SpeedTooHigh);
    CHECK_THROWS_AS(movement_boid.setVelocity(Velocity(HUGE_VAL,0.)), E_SpeedTooHigh);
    CHECK_THROWS_AS(movement_boid.setVelocity(Velocity(HUGE_VAL,HUGE_VAL)), E_SpeedTooHigh);

    CHECK_THROWS_AS(movement_boid.setVelocity(Velocity(0.,NAN)), E_SpeedTooHigh);
    CHECK_THROWS_AS(movement_boid.setVelocity(Velocity(NAN,0.)), E_SpeedTooHigh);
    CHECK_THROWS_AS(movement_boid.setVelocity(Velocity(NAN,NAN)), E_SpeedTooHigh);

    //try to set angle to nonfinites, constructors should throw
    CHECK_THROWS_AS(movement_boid.setAngle(Angle(INFINITY)), E_InvalidAngle);
    CHECK_THROWS_AS(movement_boid.setAngle(Angle(HUGE_VAL)), E_InvalidAngle);
    CHECK_THROWS_AS(movement_boid.setAngle(Angle(NAN)), E_InvalidAngle);

  }

  SUBCASE("Moving out of bounds"){
    movement_boid = Boid(Position(std::sqrt(MAX_RADIUS2) - 0.5, 0.), Velocity(1.,0.));  //put boid near edge
    movement_boid.moveBoid(1.); //move out of bounds in small step from near edge
    CHECK(movement_boid.getPosition() == Position(std::sqrt(MAX_RADIUS2 - 1), 0.));
  }

  SUBCASE("Passing nonfinites to moveBoid"){
    movement_boid = Boid(); //reset boid
    
    //passing nonfinites, should throw
    CHECK_THROWS_AS(movement_boid.moveBoid(INFINITY), E_InvalidMovementTime);
    CHECK_THROWS_AS(movement_boid.moveBoid(HUGE_VAL), E_InvalidMovementTime);
    CHECK_THROWS_AS(movement_boid.moveBoid(NAN), E_InvalidMovementTime);
  }
}


TEST_CASE("Testing velocity update"){
  Boid velocity_boid = Boid(); //spawn still boid
  
  SUBCASE("Intended behavior"){    //AGGIUNGERE TEST SULL'ANGOLO DI VISTA
    
    //updateBoidVelocity, no predators, no sight limit
    std::vector<Boid> predators0; //initialize empty vector

    velocity_boid.setPosition(Position(1., 1.));
    std::vector<Boid> boids0;
      boids0.push_back(velocity_boid);
    velocity_boid.updateBoidVelocity(boids0, predators0, 100., 1., 100., 0.5, 0.5, Angle(180.));
    CHECK(velocity_boid.getVelocity().getXVel() == 0.); //check no velocity is added
    CHECK(velocity_boid.getVelocity().getYVel() == 0.);

    std::vector<Boid> boids1;
      boids1.push_back(velocity_boid);
      boids1.push_back(Boid(Position(0.5, 0.5))); //sep boid
      boids1.push_back(Boid(Position(1.5, 0.5))); //sep boid
      boids1.push_back(Boid(Position(-1., -1.))); //non-sep, close boid
      boids1.push_back(Boid(Position(75., 75.))); //non-close boid
    velocity_boid.updateBoidVelocity(boids1, predators0, 100., 1., 100., 0.5, 0.5, Angle(180.));
    CHECK(velocity_boid.getVelocity().getXVel() == doctest::Approx(-0.33333));
    CHECK(velocity_boid.getVelocity().getYVel() == doctest::Approx(99.5));
    CHECK(velocity_boid.getAngle().getDegrees() == doctest::Approx(90.192));

    velocity_boid.setVelocity(Velocity(0., 0.)); //reset velocity
    velocity_boid.setAngle(Angle(0.)); //reset angle
    std::vector<Boid> boids2;
      boids2.push_back(velocity_boid);
      boids2.push_back(Boid(Position(0.5, 0.5), Velocity(10, -10))); //sep boid
      boids2.push_back(Boid(Position(1.5, 0.5), Velocity(-100., 0.))); //sep boid
      boids2.push_back(Boid(Position(-1., -1.), Velocity(300., -500.))); //non-sep, close boid
      boids2.push_back(Boid(Position(75., 75.), Velocity(100., 100.))); //non-close boid
    velocity_boid.updateBoidVelocity(boids2, predators0, 100., 1., 100., 0.5, 0.5, Angle(180.));
    CHECK(velocity_boid.getVelocity().getXVel() == doctest::Approx(34.667));
    CHECK(velocity_boid.getVelocity().getYVel() == doctest::Approx(14.5));
    CHECK(velocity_boid.getAngle().getDegrees() == doctest::Approx(22.698));

    velocity_boid.setVelocity(Velocity(0., 0.)); //reset velocity
    velocity_boid.setAngle(Angle(0.)); //reset angle
    std::vector<Boid> boids3;
      boids3.push_back(velocity_boid);
      boids3.push_back(Boid(Position(0.5, 0.5), Velocity(10., -10.), Angle(15.))); //sep boid
      boids3.push_back(Boid(Position(1.5, 0.5), Velocity(-100., 0.), Angle(150.))); //sep boid
      boids3.push_back(Boid(Position(-1., -1.), Velocity(300., -500.), Angle(200.))); //non-sep, close boid
      boids3.push_back(Boid(Position(75., 75.), Velocity(100., 100.), Angle(45.))); //non-close boid
    velocity_boid.updateBoidVelocity(boids3, predators0, 100., 1., 100., 0.5, 0.5, Angle(180.));
    CHECK(velocity_boid.getVelocity().getXVel() == doctest::Approx(34.667)); //check other boids' angles do not influence vel
    CHECK(velocity_boid.getVelocity().getYVel() == doctest::Approx(14.5));
    CHECK(velocity_boid.getAngle().getDegrees() == doctest::Approx(22.698));

    velocity_boid.setVelocity(Velocity(0., 0.)); //reset velocity
    velocity_boid.setAngle(Angle(0.)); //reset angle
    std::vector<Boid> boids4;
      boids4.push_back(velocity_boid);
      boids4.push_back(Boid(Position(0.5, 0.5), Velocity(10., -10.), Angle(15.))); //sep boid
      boids4.push_back(Boid(Position(1.5, 0.5), Velocity(-100., 0.), Angle(150.))); //sep boid
      boids4.push_back(Boid(Position(-1., -1.), Velocity(300., -500.), Angle(200.))); //non-sep, close boid
    velocity_boid.updateBoidVelocity(boids4, predators0, 100., 1., 100., 0.5, 0.5, Angle(180.));
    CHECK(velocity_boid.getVelocity().getXVel() == doctest::Approx(34.667)); //check non-close boid does not influence vel
    CHECK(velocity_boid.getVelocity().getYVel() == doctest::Approx(14.5));
    CHECK(velocity_boid.getAngle().getDegrees() == doctest::Approx(22.698));

    velocity_boid.setVelocity(Velocity(50., 50.)); //test with initial velocity != 0
    velocity_boid.setAngle(Angle(0.)); //reset angle
    velocity_boid.updateBoidVelocity(boids4, predators0, 100., 1., 100., 0.5, 0.5, Angle(180.)); //3 close: 2 sep, 1 non-sep boids
    CHECK(velocity_boid.getVelocity().getXVel() == doctest::Approx(59.667));
    CHECK(velocity_boid.getVelocity().getYVel() == doctest::Approx(39.5));
    CHECK(velocity_boid.getAngle().getDegrees() == doctest::Approx(33.505));

    velocity_boid.setPosition(Position(0.1, 1.)); //test with different initial position, consequently with different sep boids
    velocity_boid.setVelocity(Velocity(0., 0.)); //reset velocity
    velocity_boid.setAngle(Angle(0.)); //reset angle
    std::vector<Boid> boids5;
      boids5.push_back(velocity_boid);
      boids5.push_back(Boid(Position(0.5, 0.5), Velocity(10., -10.), Angle(15.))); //sep boid
      boids5.push_back(Boid(Position(1.5, 0.5), Velocity(-100., 0.), Angle(150.))); //non-sep, close boid
      boids5.push_back(Boid(Position(1., -1.), Velocity(300., -500.), Angle(200.))); //non-sep, close boid
    velocity_boid.updateBoidVelocity(boids5, predators0, 100., 1., 100., 0.5, 0.5, Angle(180.));
    CHECK(velocity_boid.getVelocity().getXVel() == doctest::Approx(-4.55));
    CHECK(velocity_boid.getVelocity().getYVel() == doctest::Approx(-35.5));
    CHECK(velocity_boid.getAngle().getDegrees() == doctest::Approx(262.696));

    velocity_boid.setPosition(Position(0.1, 1.));
    velocity_boid.setVelocity(Velocity(50., 50.)); //check with initial velocity != 0
    velocity_boid.setAngle(Angle(0.)); //reset angle
    velocity_boid.updateBoidVelocity(boids5, predators0, 100., 1., 100., 0.5, 0.5, Angle(180.)); //3 close: 1 sep, 2 non-sep boids
    CHECK(velocity_boid.getVelocity().getXVel() == doctest::Approx(20.45));
    CHECK(velocity_boid.getVelocity().getYVel() == doctest::Approx(-10.5));
    CHECK(velocity_boid.getAngle().getDegrees() == doctest::Approx(332.822));

    velocity_boid.setPosition(Position(0.1, 1.)); 
    velocity_boid.setVelocity(Velocity(50., 50.));
    velocity_boid.setAngle(Angle(100.)); //test with initial angle != 0
    velocity_boid.updateBoidVelocity(boids5, predators0, 100., 1., 100., 0.5, 0.5, Angle(180.)); //3 close: 1 sep, 2 non-sep boids
    CHECK(velocity_boid.getVelocity().getXVel() == doctest::Approx(20.45)); //check initial angle does not influence results
    CHECK(velocity_boid.getVelocity().getYVel() == doctest::Approx(-10.5));
    CHECK(velocity_boid.getAngle().getDegrees() == doctest::Approx(332.822));

    velocity_boid.setPosition(Position(707., 707.)); //bring boid near edge
    velocity_boid.setVelocity(Velocity(0., 0.)); //reset velocity
    velocity_boid.setAngle(Angle(0.)); //reset angle
    velocity_boid.updateBoidVelocity(boids5, predators0, 100., 1., 100., 0.5, 0.5, Angle(180.)); //no close boids, vel == edge_vel
    CHECK(velocity_boid.getVelocity().getXVel() == doctest::Approx(-2.0345)); //near edge edge_vel becomes significant
    CHECK(velocity_boid.getVelocity().getYVel() == doctest::Approx(-2.0345));
    CHECK(velocity_boid.getAngle().getDegrees() == doctest::Approx(225.)); //boid heads towards opposite direction


    //updateBoidVelocity, predators, no sight limit
    velocity_boid.setPosition(Position(1., 1.));
    velocity_boid.setVelocity(Velocity(0., 0.)); //reset velocity
    std::vector<Boid> predators1;
      predators1.push_back(Boid(Position(0.5, 0.5))); //sep predator
    velocity_boid.updateBoidVelocity(boids0, predators1, 100., 1., 100., 0.5, 0.5, Angle(180.));
    CHECK(velocity_boid.getVelocity().getXVel() == 50.0); //check only added velocity is sep_vel due to predator: the boid flees from it
    CHECK(velocity_boid.getVelocity().getYVel() == 50.0);
    CHECK(velocity_boid.getAngle().getDegrees() == 45.);

    velocity_boid.setVelocity(Velocity(0., 0.)); //reset velocity
    velocity_boid.setAngle(Angle(0.)); //reset angle
    std::vector<Boid> boids6;
      boids6.push_back(velocity_boid);
      boids6.push_back(Boid(Position(1.5, 0.5))); //sep boid
    velocity_boid.updateBoidVelocity(boids6, predators1, 100., 1., 100., 0.5, 0.5, Angle(180.));
    CHECK(velocity_boid.getVelocity().getXVel() == doctest::Approx(0.250));
    CHECK(velocity_boid.getVelocity().getYVel() == doctest::Approx(99.75));
    CHECK(velocity_boid.getAngle().getDegrees() == doctest::Approx(89.856));

    
    //update updateBoidVelocity, sight angle
    velocity_boid.setVelocity(Velocity(0., 0.)); //reset velocity
    velocity_boid.setAngle(Angle(0.)); //reset angle
    std::vector<Boid> boids7;
      boids7.push_back(velocity_boid);
      boids7.push_back(Boid(Position(1.3, 1.))); //sep boid in sight
      boids7.push_back(Boid(Position(2., 1.3))); //non-sep, close boid in sight
      boids7.push_back(Boid(Position(0.9, 1.))); //sep boid, not in sight
    velocity_boid.updateBoidVelocity(boids7, predators0, 100., 1., 100., 0.5, 0.5, Angle(45.)); //0 is in sight range
    CHECK(velocity_boid.getVelocity().getXVel() == doctest::Approx(-29.675)); //check third boid is not seen
    CHECK(velocity_boid.getVelocity().getYVel() == doctest::Approx(0.0750));
    CHECK(velocity_boid.getAngle().getDegrees() == doctest::Approx(179.855));

    velocity_boid.setVelocity(Velocity(0., 0.)); //reset velocity
    velocity_boid.setAngle(Angle(0.)); //reset angle
    velocity_boid.updateBoidVelocity(boids7, predators1, 100., 1., 100., 0.5, 0.5, Angle(45.)); //0 is in sight range
    CHECK(velocity_boid.getVelocity().getXVel() == doctest::Approx(20.325)); //check predator is seen, though not in sight range
    CHECK(velocity_boid.getVelocity().getYVel() == doctest::Approx(50.075));
    CHECK(velocity_boid.getAngle().getDegrees() == doctest::Approx(67.908));

    velocity_boid.setVelocity(Velocity(0., 0.)); //reset velocity
    velocity_boid.setAngle(Angle(90.)); //test with different initial angle
    velocity_boid.updateBoidVelocity(boids7, predators0, 100., 1., 100., 0.5, 0.5, Angle(45.)); //0 is not in sight range
    CHECK(velocity_boid.getVelocity().getXVel() == doctest::Approx(0.)); //check no boid is seen
    CHECK(velocity_boid.getVelocity().getYVel() == doctest::Approx(0.));

    velocity_boid.setVelocity(Velocity(0., 0.)); //reset velocity
    velocity_boid.setAngle(Angle(90.)); //test with different initial angle
    velocity_boid.updateBoidVelocity(boids7, predators0, 100., 1., 100., 0.5, 0.5, Angle(100.)); //0 is in sight range
    CHECK(velocity_boid.getVelocity().getXVel() == doctest::Approx(-19.8)); //check no boid is seen
    CHECK(velocity_boid.getVelocity().getYVel() == doctest::Approx(0.05));
    CHECK(velocity_boid.getAngle().getDegrees() == doctest::Approx(179.855));


    //updateBoidVelocity, different flocks
    velocity_boid.setFlock(1);
    velocity_boid.setVelocity(Velocity(0., 0.)); //reset velocity
    velocity_boid.setAngle(Angle(0.)); //reset angle
    std::vector<Boid> boids8;
      boids8.push_back(velocity_boid);
      boids8.push_back(Boid(Position(1.5, 1.5), Velocity(1., -1.), Angle(225.), 2));
      boids8.push_back(Boid(Position(0., 1.), Velocity(0., 10.), Angle(15.), 3));
    velocity_boid.updateBoidVelocity(boids8, predators0, 100., 1., 100., 0.5, 0.5, Angle(180.));
    CHECK(velocity_boid.getVelocity().getXVel() == 0.); //check other flocks' boids are ignored
    CHECK(velocity_boid.getVelocity().getYVel() == 0.);

      boids8.push_back(Boid(Position(1., 1.5), Velocity(1., -1.), Angle(90.), 1)); //add boid in same flock
    velocity_boid.updateBoidVelocity(boids8, predators0, 100., 1., 100., 0.5, 0.5, Angle(180.));
    CHECK(velocity_boid.getVelocity().getXVel() == 0.5); //check new boid is seen
    CHECK(velocity_boid.getVelocity().getYVel() == -50.25);
    CHECK(velocity_boid.getAngle().getDegrees() == doctest::Approx(270.570));

    velocity_boid.setVelocity(Velocity(0., 0.)); //reset velocity
    velocity_boid.setAngle(Angle(270.)); //change angle
    velocity_boid.updateBoidVelocity(boids8, predators0, 100., 1., 100., 0.5, 0.5, 60.);
    CHECK(velocity_boid.getVelocity().getXVel() == 0.); //check same flock's boid is out of sight range
    CHECK(velocity_boid.getVelocity().getYVel() == 0.);


    //updatePredatorVelocity, no sight limit
    velocity_boid.setPosition(Position(1., 1.));
    velocity_boid.setVelocity(Velocity(0., 0.)); //reset velocity
      predators0.push_back(velocity_boid);
      boids0.pop_back(); //remove velocity_boid from boids0, which is now empty
    velocity_boid.updatePredatorVelocity(predators0, boids0, 100., 1., 100., 0.5, 0.5, Angle(180.));
    CHECK(velocity_boid.getVelocity().getXVel() == 0.); //check velocity doesn't change
    CHECK(velocity_boid.getVelocity().getYVel() == 0.);

      predators1.push_back(velocity_boid);
    velocity_boid.updatePredatorVelocity(predators1, boids0, 100., 1., 100., 0.5, 0.5, Angle(180.));
    CHECK(velocity_boid.getVelocity().getXVel() == 50.); //check sep_vel due to other predator is added
    CHECK(velocity_boid.getVelocity().getYVel() == 50.);
    CHECK(velocity_boid.getAngle().getDegrees() == 45.); //check predator heads towards opposite direction

    velocity_boid.setVelocity(Velocity(0., 0.)); //reset velocity
    std::vector<Boid> boids9;
      boids9.push_back(Boid(Position(1.5, 1.5)));
    velocity_boid.updatePredatorVelocity(predators0, boids9, 100., 1., 100., 0.5, 0.5, Angle(180.));
    CHECK(velocity_boid.getVelocity().getXVel() == 0.25); //check cohes_vel due to close boid is added
    CHECK(velocity_boid.getVelocity().getYVel() == 0.25);
    CHECK(velocity_boid.getAngle().getDegrees() == 45.); //check predator chases the boid

    velocity_boid.setVelocity(Velocity(0., 0.)); //reset velocity
    std::vector<Boid> boids10;
      boids10.push_back(Boid(Position(1.5, 1.5), Velocity(1., -1.)));
    velocity_boid.updatePredatorVelocity(predators0, boids10, 100., 1., 100., 0.5, 0.5, Angle(180.));
    CHECK(velocity_boid.getVelocity().getXVel() == 0.75); //check align_vel and cohes_vel due to close boid are added
    CHECK(velocity_boid.getVelocity().getYVel() == -0.25);
    CHECK(velocity_boid.getAngle().getDegrees() == doctest::Approx(341.565));
    
    velocity_boid.setVelocity(Velocity(0., 0.)); //reset velocity
    velocity_boid.updatePredatorVelocity(predators1, boids10, 100., 1., 100., 0.5, 0.5, Angle(180.));
    CHECK(velocity_boid.getVelocity().getXVel() == 50.75); //check with one other predator and one boid
    CHECK(velocity_boid.getVelocity().getYVel() == 49.75);
    CHECK(velocity_boid.getAngle().getDegrees() == doctest::Approx(44.430));

    velocity_boid.setVelocity(Velocity(10., -10.)); //test with initial velocity != 0
    velocity_boid.updatePredatorVelocity(predators1, boids10, 100., 1., 100., 0.5, 0.5, Angle(180.));
    CHECK(velocity_boid.getVelocity().getXVel() == 55.75);
    CHECK(velocity_boid.getVelocity().getYVel() == 44.75);
    CHECK(velocity_boid.getAngle().getDegrees() == doctest::Approx(38.754));


    //updatePredatorVelocity, sight angle
    velocity_boid.setVelocity(Velocity(0., 0.)); //reset velocity
    velocity_boid.setAngle(Angle(0.)); //reset angle
    velocity_boid.updatePredatorVelocity(predators1, boids9, 100., 1., 100., 0.5, 0.5, Angle(90.)); //one sep boid in sight
    CHECK(velocity_boid.getVelocity().getXVel() == 0.25);
    CHECK(velocity_boid.getVelocity().getYVel() == 0.25);
    CHECK(velocity_boid.getAngle().getDegrees() == 45.);

    velocity_boid.setVelocity(Velocity(0., 0.)); //reset velocity
    velocity_boid.setAngle(Angle(135.)); //reset angle
    velocity_boid.updatePredatorVelocity(predators1, boids9, 100., 1., 100., 0.5, 0.5, Angle(100.)); //one sep boid and one other predator in sight
    CHECK(velocity_boid.getVelocity().getXVel() == 50.25);
    CHECK(velocity_boid.getVelocity().getYVel() == 50.25);
    CHECK(velocity_boid.getAngle().getDegrees() == 45.);

  }
}