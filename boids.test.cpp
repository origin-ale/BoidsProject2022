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
    CHECK_THROWS_AS(Boid toofar_boid = Boid(Position(MAX_RADIUS2,MAX_RADIUS2), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
  }

  SUBCASE("Overflowing OoB"){
    CHECK_THROWS_AS(Boid hugefar_boid1 = Boid(Position(HUGE_VAL,0.), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
    CHECK_THROWS_AS(Boid hugefar_boid2 = Boid(Position(0.,HUGE_VAL), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
    CHECK_THROWS_AS(Boid hugefar_boid3 = Boid(Position(HUGE_VAL,HUGE_VAL), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
  }

  SUBCASE("Infinite OoB"){ //might not work
    CHECK_THROWS_AS(Boid inffar_boid1 = Boid(Position(INFINITY,0.), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
    CHECK_THROWS_AS(Boid inffar_boid2 = Boid(Position(0.,INFINITY), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
    CHECK_THROWS_AS(Boid inffar_boid3 = Boid(Position(INFINITY,INFINITY), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
  } 

  SUBCASE("NaN OoB"){ //probably won't work
    CHECK_THROWS_AS(Boid nanfar_boid1 = Boid(Position(NAN,0.), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
    CHECK_THROWS_AS(Boid nanfar_boid2 = Boid(Position(0.,NAN), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
    CHECK_THROWS_AS(Boid nanfar_boid3 = Boid(Position(NAN,NAN), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
  } 

  SUBCASE("Mixed OoB"){
    CHECK_THROWS_AS(Boid mixfar_boid1 = Boid(Position(INFINITY,HUGE_VAL), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
    CHECK_THROWS_AS(Boid mixfar_boid2 = Boid(Position(HUGE_VAL,INFINITY), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
    CHECK_THROWS_AS(Boid mixfar_boid3 = Boid(Position(INFINITY,NAN), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
    CHECK_THROWS_AS(Boid mixfar_boid4 = Boid(Position(NAN,INFINITY), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
    CHECK_THROWS_AS(Boid mixfar_boid5 = Boid(Position(HUGE_VAL,NAN), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
    CHECK_THROWS_AS(Boid mixfar_boid6 = Boid(Position(NAN,HUGE_VAL), Velocity(0.,0.), Angle(0.)), E_OutOfBounds);
  } 
}

TEST_CASE("Testing speed limit"){ // try getting boid to speed higher than MAX_SPEED

  SUBCASE("Finite speed break"){
    CHECK_THROWS_AS(Boid toofast_boid = Boid(Position(0.,0.), Velocity(MAX_SPEED2,MAX_SPEED2), Angle(0.)), E_SpeedTooHigh);
  }

  SUBCASE("Overflowing speed break"){
    CHECK_THROWS_AS(Boid hugefast_boid1 = Boid(Position(0.,0.), Velocity(HUGE_VAL,0.), Angle(0.)), E_SpeedTooHigh);
    CHECK_THROWS_AS(Boid hugefast_boid2 = Boid(Position(0.,0.), Velocity(0.,HUGE_VAL), Angle(0.)), E_SpeedTooHigh);
    CHECK_THROWS_AS(Boid hugefast_boid3 = Boid(Position(0.,0.), Velocity(HUGE_VAL,HUGE_VAL), Angle(0.)), E_SpeedTooHigh);
  }

  SUBCASE("Infinite speed break"){
    CHECK_THROWS_AS(Boid inffast_boid1 = Boid(Position(0.,0.), Velocity(INFINITY,0.), Angle(0.)), E_SpeedTooHigh);
    CHECK_THROWS_AS(Boid inffast_boid2 = Boid(Position(0.,0.), Velocity(0.,INFINITY), Angle(0.)), E_SpeedTooHigh);
    CHECK_THROWS_AS(Boid inffast_boid3 = Boid(Position(0.,0.), Velocity(INFINITY,INFINITY), Angle(0.)), E_SpeedTooHigh);
  }

  SUBCASE("NaN speed break"){
    CHECK_THROWS_AS(Boid nanfast_boid1 = Boid(Position(0.,0.), Velocity(NAN,0.), Angle(0.)), E_SpeedTooHigh);
    CHECK_THROWS_AS(Boid nanfast_boid2 = Boid(Position(0.,0.), Velocity(0.,NAN), Angle(0.)), E_SpeedTooHigh);
    CHECK_THROWS_AS(Boid nanfast_boid3 = Boid(Position(0.,0.), Velocity(NAN,NAN), Angle(0.)), E_SpeedTooHigh);
  }
  SUBCASE("Mixed speed break"){
    CHECK_THROWS_AS(Boid mixfast_boid1 = Boid(Position(0.,0.), Velocity(INFINITY,HUGE_VAL), Angle(0.)), E_SpeedTooHigh);
    CHECK_THROWS_AS(Boid mixfast_boid2 = Boid(Position(0.,0.), Velocity(HUGE_VAL,INFINITY), Angle(0.)), E_SpeedTooHigh);
    CHECK_THROWS_AS(Boid mixfast_boid3 = Boid(Position(0.,0.), Velocity(INFINITY,NAN), Angle(0.)), E_SpeedTooHigh);
    CHECK_THROWS_AS(Boid mixfast_boid4 = Boid(Position(0.,0.), Velocity(NAN,INFINITY), Angle(0.)), E_SpeedTooHigh);
    CHECK_THROWS_AS(Boid mixfast_boid5 = Boid(Position(0.,0.), Velocity(HUGE_VAL,NAN), Angle(0.)), E_SpeedTooHigh);
    CHECK_THROWS_AS(Boid mixfast_boid6 = Boid(Position(0.,0.), Velocity(NAN,HUGE_VAL), Angle(0.)), E_SpeedTooHigh);
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
    CHECK_THROWS_AS(Boid infrot_boid = Boid(Position(0.,0.), Velocity(0.,0.), Angle(INFINITY)), E_InvalidAngle);
    CHECK_THROWS_AS(Boid hugerot_boid = Boid(Position(0.,0.), Velocity(0.,0.), Angle(HUGE_VAL)), E_InvalidAngle);
    CHECK_THROWS_AS(Boid nanrot_boid = Boid(Position(0.,0.), Velocity(0.,0.), Angle(NAN)), E_InvalidAngle);
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
    movement_boid = Boid(Position(0.,0.), Velocity(std::sqrt(MAX_SPEED2),0.)); //put boid in origin, with maximum allowed speed
    movement_boid.moveBoid((std::sqrt(MAX_RADIUS2)/std::sqrt(MAX_SPEED2)) + 10); //move out of bounds in one large step
    CHECK(movement_boid.getPosition() == Position(0.,0.)); //should not have moved

    movement_boid = Boid(Position(std::sqrt(MAX_RADIUS2) - 0.5, 0.), Velocity(1.,0.));  //put boid near edge
    movement_boid.moveBoid(1.); //move out of bounds in small step from near edge
    CHECK(movement_boid.getPosition() == Position(std::sqrt(MAX_RADIUS2) - 0.5, 0.)); //should not have moved
  }

  SUBCASE("Passing nonfinites to moveBoid"){
    movement_boid = Boid(); //reset boid
    
    //passing nonfinites, should throw
    CHECK_THROWS_AS(movement_boid.moveBoid(INFINITY), E_InvalidMovementTime);
    CHECK_THROWS_AS(movement_boid.moveBoid(HUGE_VAL), E_InvalidMovementTime);
    CHECK_THROWS_AS(movement_boid.moveBoid(NAN), E_InvalidMovementTime);
  }
}