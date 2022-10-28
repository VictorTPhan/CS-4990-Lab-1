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
  ArrayList<PVector> points;
  
  PVector origin;
  int currentPointIndex = 0;
  boolean finished = false;
  
  float defaultFinishRadius = 20;
  float finishRadius = 20;
 
  float angleToNextPoint;
  float speedDampen = 3;

  //constructor for boid
  Boid(PVector position, float heading, float max_speed, float max_rotational_speed, float acceleration, float rotational_acceleration)
  {
    this.kinematic = new KinematicMovement(position, heading, max_speed, max_rotational_speed);
    this.last_crumb = millis();
    this.max_acceleration = acceleration;
    this.max_rotational_acceleration = rotational_acceleration;
  }
  
  //assuming the boid is following a path, get the distance from this point to the next
  //if you're on the last point, then it returns a sentinel value;
  float distanceToNextPoint(int index) {
    int nextIndex = index+1;
    if (nextIndex >= points.size())
      return -1;
    else {
      PVector currentPoint = points.get(index);
      PVector nextPoint = points.get(nextIndex);
      print("From " + index + " to " + nextIndex + ": ");
      return distanceBetween(currentPoint, nextPoint);
    }
  }
  
  //assuming the boid is following a path, determine if the boid needs to be stricter in its finishing radius
  boolean shouldChangeFinishingRadius(int currentIndex)
  {
    float result = distanceToNextPoint(currentIndex);
    println(result);
    if (result < 0 || result > defaultFinishRadius + 20) return false; //<>//
    else return true; //if the distance is closer than the radius allows
  }

  //a helper function to draw a line. Only for debugging.
  void drawAngledLine(float radian, String tag)
  {
    int lineLength = 200;
    float y1 = sin(radian) * lineLength;
    float x1 = cos(radian) * lineLength;
    line(kinematic.position.x, kinematic.position.y, kinematic.position.x + x1, kinematic.position.y + y1);

    text(tag + " " + radian, kinematic.position.x + x1, kinematic.position.y + y1);
  }

  //a helper function to determine the sign of a number
  int signOf(float n)
  {
    return (n > 0)? 1: -1;
  }

  //gets the distance between two PVectors
  float distanceBetween(PVector A, PVector B)
  {
    return sqrt(pow(A.x - B.x, 2) + pow(A.y - B.y, 2));
  }

  //finds the shortest radian between radianA to radianB.
  //if returns negative, then A is to the left of B.
  float shortestRadianBetween(float radianA, float radianB)
  {
    //find the shortest radian distance between target and you (that way you don't do a giant rotation and mess up your speed)
    //this value is between -pi and pi
    float shortestRadianDistance = normalize_angle(radianA) - normalize_angle(radianB);
    float counter = signOf(shortestRadianDistance) * -1 * (TWO_PI - abs(shortestRadianDistance));
    return (abs(shortestRadianDistance) < abs(counter))? shortestRadianDistance: counter;
  }

  //determines if the boid is approaching the final point in their points list.
  boolean movingToLastPoint() {
    return (points==null || points.size() == 0) || (points != null && currentPointIndex == points.size()-1);
  }

  //determines the angle between two lines, AB, and BC.
  float getAngleBetween(PVector A, PVector B, PVector C) {
    PVector AB = PVector.sub(B, A);
    PVector BC = PVector.sub(C, B);
    float theta = PVector.angleBetween(AB, BC);
    println(theta * 52.97);
    return theta;
  }

  //assuming the boid is following a path, determine the angle that it has to turn when it approaches its target.
  void getAngleToNextPoint() {
    angleToNextPoint = getAngleBetween(points.get(currentPointIndex), points.get(currentPointIndex + 1), points.get(currentPointIndex + 2));
  }

  void update(float dt)
  {
    text(kinematic.getHeading(), kinematic.position.x, kinematic.position.y + 20);

    if (points != null) {
      for (int i = 0; i<points.size(); i++)
      {
        text(i, points.get(i).x + 10, points.get(i).y + 10);
      }
    }
    
    if (finished) { //if it has completed following its path
      //if its still moving by the time it reaches its target, decelerate quickly
      if (kinematic.speed > 0) {
        //kinematic.speed / kinematic.max_speed always returns a value from -1 to 1
        //once the kinematic speed goes under 0, it will go forward a bit, then go back, and will eventually oscillate at 0
        //the same principle is applied for the rotational velocity
        kinematic.increaseSpeed(-kinematic.speed / kinematic.max_speed, kinematic.rotational_velocity * -1);
      }
    }
    else if (target != null)
    {
      circle(target.x, target.y, finishRadius);

      ///// ROTATION

      //grab direction of target
      float targetRadian = atan2(target.y - kinematic.position.y, target.x - kinematic.position.x);

      //find the shortest radian distance between target and you
      //this value is between -pi and pi
      float shortestRadianDistance = shortestRadianBetween(targetRadian, kinematic.getHeading());
      boolean clockwise = shortestRadianDistance >= 0;

      float rotation = signOf(shortestRadianDistance) * max_rotational_acceleration;

      //first expression is self explanatory, second expressed will determine the slowdown marker
      //given acceleration, a destination angle, and a radian distance to a destination angle, you could calculate the radian angle at which you ought to slow down
      //this radian is calculated by doing targetRadian - (kinematic.rotational_velocity/2)
      //therefore, if you're going too fast towards a target, we'll check if you've crossed this threshold yet.
      //as you approach the target, you will slow down and keep on track with this invisible marker until you arrive at your target radian
      boolean clockwiseAndApproaching = clockwise && shortestRadianBetween(kinematic.getHeading(), targetRadian - (kinematic.rotational_velocity/2)) > 0;
      boolean counterClockwiseAndApproaching = !clockwise && shortestRadianBetween(kinematic.getHeading(), targetRadian - (kinematic.rotational_velocity/2)) < 0;
      boolean approaching = clockwiseAndApproaching || counterClockwiseAndApproaching;

      //if you're coming close, then slow down!
      if (approaching)
      {
        rotation = signOf(rotation) * -1;
      }
      
      ///// POSITION
      
      //determine distance from target to start and target to boid
      float targetToOrigin = distanceBetween(target, origin);
      float targetToBoid = distanceBetween(target, kinematic.position);
      
      //we'll first assume that we should go as fast as possible
      float speed = max_acceleration;
           
      float progress = targetToBoid / targetToOrigin;  //closer it is to 0, the closer the boid is to the target, expressed from 0 to 1
      float velocityAmount = kinematic.speed/kinematic.max_speed; //how much potential speed is being used, expressed from -1 to 1

      float brakeMultiplier = ((min(HALF_PI, abs(angleToNextPoint)))/2); //apply a variable braking force if the next angle is at most 90 degrees wide
      float slowdownThreshold = pow(velocityAmount * brakeMultiplier, 2); //we'll set a marker at which the boid ought to slow down, expressed from 0 to 1 
      
      //since progress and slowdownThreshold are in range [0,1], we can compare our progress to see if we are going too fast
      //if so, slow down.
      if (progress < slowdownThreshold)
      {
        speed = -max_acceleration;
      }
      
      //allows the boid to make really tight and concise turns, but only when turning and if moving
      if (abs(kinematic.rotational_velocity) > 1 && kinematic.speed > 10) {
        speed -= (abs(shortestRadianDistance)/PI * speed) * 2 * speedDampen;
      }
      
      //it's pretty much done at this point
      if (targetToBoid < finishRadius) {
        println("Finished with point: " + currentPointIndex);  //<>//
        if (movingToLastPoint())
        {
          finished = true;
        }
        //if there are more points to go to
        else if (points != null && (points.size() > 2 || currentPointIndex < points.size()-1))
        {
          //if the next point is really close, then set your finishing radius smaller so that you can actually move there
          if (shouldChangeFinishingRadius(currentPointIndex))
            finishRadius = defaultFinishRadius/3;
          else finishRadius = defaultFinishRadius;
          
          if (currentPointIndex < points.size()-2)
          {
            getAngleToNextPoint();
          }

          if (!movingToLastPoint())
          {
            //set target to the next point in your points list
            currentPointIndex++;
            seek(points.get(currentPointIndex));
            println("Moving to point: " + currentPointIndex);
          }
        }
      }

      //for the last point, slowing down is much different because you're fully stopping here
      //we'll basically set a marker to be velocityAmount^2, with no braking multiplier as there is no angle to turn from
      //simply comparing to velocityAmount would cause the boid to slow down earlier than ""naturally"" and it's quite slow
      if (movingToLastPoint())
      {
        if (progress < velocityAmount*velocityAmount) {
          speed = -max_acceleration;
        }
      }


      kinematic.increaseSpeed(speed * progress, rotation * dt);

      drawAngledLine(kinematic.getHeading(), "heading");
      drawAngledLine(targetRadian, "target angle");

      text("Shortest Radian Distance: " + shortestRadianDistance + TWO_PI, kinematic.position.x + 50, kinematic.position.y + 20);
      text("Rotational Velocity: " + kinematic.rotational_velocity, kinematic.position.x + 50, kinematic.position.y + 35);
      text("Rotation: " + rotation, kinematic.position.x + 50, kinematic.position.y + 50);
      text("Clockwise: " + clockwise, kinematic.position.x + 50, kinematic.position.y + 65);
      text("Approaching: " + approaching, kinematic.position.x + 50, kinematic.position.y + 80);
      text("Positional Velocity: " + speed, kinematic.position.x + 50, kinematic.position.y + 95);
      text("Progress: " + progress, kinematic.position.x + 50, kinematic.position.y + 110);
      text("Velocity Amount: " + velocityAmount, kinematic.position.x + 50, kinematic.position.y +125);
      text("Target to Boid: " + targetToBoid, kinematic.position.x + 50, kinematic.position.y + 140);
      text("Angle to Next Point: " + angleToNextPoint, kinematic.position.x + 50, kinematic.position.y + 155);
      text("Kinematic Velocity: " + kinematic.speed, kinematic.position.x + 50, kinematic.position.y + 170);
      text("Braking Multiplier: " + brakeMultiplier, kinematic.position.x + 50, kinematic.position.y + 185);
    }

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
    origin = kinematic.position;
    
    println("Origin: " + origin);
    println("Target: " + target);
  }

  void follow(ArrayList<PVector> waypoints)
  {
    finished = false;
    points = waypoints;
    currentPointIndex = 0;
    if (points.size() > 2) angleToNextPoint = getAngleBetween(kinematic.position, points.get(0), points.get(1));
    seek(points.get(currentPointIndex));
  }
}
