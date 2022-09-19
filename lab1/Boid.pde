/// In this file, you will have to implement seek and waypoint-following
/// The relevant locations are marked with "TODO"

class Crumb
{
  PVector position;
  Crumb(PVector position)
  {
    this.position = position;
  }
  void draw()
  {
    fill(255);
    noStroke();
    circle(this.position.x, this.position.y, CRUMB_SIZE);
  }
}

class Boid
{
  Crumb[] crumbs = {};
  int last_crumb;
  float max_acceleration;
  float max_rotational_acceleration;
  KinematicMovement kinematic;
  PVector target;

  Boid(PVector position, float heading, float max_speed, float max_rotational_speed, float acceleration, float rotational_acceleration)
  {
    this.kinematic = new KinematicMovement(position, heading, max_speed, max_rotational_speed);
    this.last_crumb = millis();
    this.max_acceleration = acceleration;
    this.max_rotational_acceleration = rotational_acceleration;
  }

  float targetRadian;
  float startingRadian;    
  
  float halfwayCheckPoint;
  float stoppingPoint;
  float correctingPoint;
  float correctingPoint2;
  
  float halfwayRadian;
  float stoppingRadian;
  float correctingRadian;
  float correctingRadian2;
  
  float amountToRotate;
  boolean clockwise;
  boolean turnComplete;
  String rotationState;
  void drawAngledLine(float radian, String tag)
  {
    int lineLength = 200;
    float y1 = sin(radian) * lineLength;
    float x1 = cos(radian) * lineLength;
    line(kinematic.position.x, kinematic.position.y, kinematic.position.x + x1, kinematic.position.y + y1);
    
    text(tag + " " + radian, kinematic.position.x + x1, kinematic.position.y + y1);
  }

  void update(float dt)
  {
    text(kinematic.getHeading(), kinematic.position.x, kinematic.position.y + 20);
    
    if (target != null)
    {
      drawAngledLine(targetRadian, "end position");
      drawAngledLine(startingRadian, "start position");
      drawAngledLine(halfwayRadian, "midway point");
      drawAngledLine(stoppingRadian, "stopping point");
      drawAngledLine(correctingRadian, "correcting point");
      drawAngledLine(correctingRadian2, "rebound point");
      
      float rotation = amountToRotate / PI;
      if (!clockwise) rotation *= -1;
      rotation *= max_acceleration;
      
      //decceleration
      float distanceSoFar = kinematic.getHeading() - startingRadian;
      float counter = TWO_PI - distanceSoFar;
      if (counter < abs(distanceSoFar)) distanceSoFar = counter;
      rotationState = "accelerating";
      
      if (abs(distanceSoFar) > abs(halfwayCheckPoint)) {
        rotation *= -max_acceleration;
        rotationState = "deccelerating";
      }
      
      if (abs(distanceSoFar) > abs(stoppingPoint)) {
        turnComplete = true;
        rotation = 0;
        rotationState = "finished";
      }
      
      if (turnComplete)
      {
        if (clockwise) {
          if (kinematic.getHeading() > correctingRadian) {
            rotation = -0.2;
            rotationState = "correcting";
          }
          else if (kinematic.rotational_velocity < 0 && kinematic.getHeading() < correctingRadian2) {
            rotation = 0.2;
            rotationState = "correcting back";
          }
        }
        else {
          if (kinematic.getHeading() < correctingRadian) {
            rotation = 0.2;
            rotationState = "correcting";
          }
          else if (kinematic.rotational_velocity > 0 && kinematic.getHeading() > correctingRadian2) {
            rotation = -0.2;
            rotationState = "correcting back";
          }
        }
      }

      //2 parameters: positional velocity and rotational velocity
      kinematic.increaseSpeed(0 * dt, rotation * dt);
      //kinematic.increaseSpeed(0 * dt, 0 * dt);

      text("Going clockwise: " + clockwise, kinematic.position.x + 50, kinematic.position.y + 15);
      text("Rotational Velocity: " + kinematic.rotational_velocity, kinematic.position.x + 50, kinematic.position.y + 30);
      text("Acceleration this frame: " + rotation, kinematic.position.x + 50, kinematic.position.y + 60);
      text("Amount to rotate: " + amountToRotate, kinematic.position.x + 50, kinematic.position.y + 45);
      text("Distance so far: " + distanceSoFar, kinematic.position.x + 50, kinematic.position.y + 105);
      text("% Distance: " + distanceSoFar/amountToRotate * 100, kinematic.position.x + 50, kinematic.position.y + 75);
      text("Rotation State: " + rotationState, kinematic.position.x + 50, kinematic.position.y + 90);
    }

    //go 3 times faster and rotate 100000 times faster
    //kinematic.increaseSpeed(3*dt, 100000 * dt);

    // place crumbs, do not change
    if (LEAVE_CRUMBS && (millis() - this.last_crumb > CRUMB_INTERVAL))
    {
      this.last_crumb = millis();
      this.crumbs = (Crumb[])append(this.crumbs, new Crumb(this.kinematic.position));
      if (this.crumbs.length > MAX_CRUMBS)
        this.crumbs = (Crumb[])subset(this.crumbs, 1);
    }

    // do not change
    this.kinematic.update(dt);

    draw();
  }

  void draw()
  {
    for (Crumb c : this.crumbs)
    {
      c.draw();
    }

    fill(255);
    noStroke();
    float x = kinematic.position.x;
    float y = kinematic.position.y;
    float r = kinematic.heading;
    circle(x, y, BOID_SIZE);
    // front
    float xp = x + BOID_SIZE*cos(r);
    float yp = y + BOID_SIZE*sin(r);

    // left
    float x1p = x - (BOID_SIZE/2)*sin(r);
    float y1p = y + (BOID_SIZE/2)*cos(r);

    // right
    float x2p = x + (BOID_SIZE/2)*sin(r);
    float y2p = y - (BOID_SIZE/2)*cos(r);
    triangle(xp, yp, x1p, y1p, x2p, y2p);
  }

  void seek(PVector target)
  {
    this.target = target;
    
    turnComplete = false;
    startingRadian = kinematic.heading;
    
    targetRadian = atan2(kinematic.position.y - target.y, kinematic.position.x - target.x) + PI;
    amountToRotate = targetRadian - startingRadian;
    println("amount to rotate: " + amountToRotate);

    float counter = TWO_PI - abs(amountToRotate);
    println("counter: " + counter);
    
    clockwise = targetRadian > kinematic.getHeading();
    if (counter < abs(amountToRotate)) {
      amountToRotate = counter;
      clockwise = !clockwise;
    }
    
    println("clockwise: " + clockwise);
        
    amountToRotate = abs(amountToRotate);
    println("final amount to rotate: " + amountToRotate);
    
    float halfWayMultiplier = 0.47;
    float stoppingPointMultiplier = 0.93;
    float correctingPointMultiplier = 1.02;
    
    halfwayCheckPoint = amountToRotate * halfWayMultiplier;
    stoppingPoint = amountToRotate * stoppingPointMultiplier;
    correctingPoint = amountToRotate * correctingPointMultiplier;
    correctingPoint2 = amountToRotate * (correctingPointMultiplier - 1);
    println("halfway amount to rotate: " + halfwayCheckPoint);
    println("stopping amount to rotate: " + stoppingPoint);
    println("correcting point: " + correctingPoint);
    
    if (clockwise) {
      halfwayRadian = normalize_angle(startingRadian + halfwayCheckPoint);
      stoppingRadian = normalize_angle(startingRadian + stoppingPoint);
      correctingRadian = normalize_angle(startingRadian + correctingPoint);
      correctingRadian2 = normalize_angle(targetRadian - (amountToRotate * (correctingPointMultiplier - 1)));
    }
    else {
      halfwayRadian = normalize_angle(targetRadian + (amountToRotate * (1-halfWayMultiplier)));
      stoppingRadian = normalize_angle(targetRadian + (amountToRotate * (1-stoppingPointMultiplier)));
      correctingRadian = normalize_angle(targetRadian - (amountToRotate * (correctingPointMultiplier - 1)));
      correctingRadian2 = normalize_angle(targetRadian + (amountToRotate * (correctingPointMultiplier - 1)));
    }
  }

  void follow(ArrayList<PVector> waypoints)
  {
    // TODO: change to follow *all* waypoints
    this.target = waypoints.get(0);
  }
}
