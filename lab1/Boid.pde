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
  int currentPointIndex = 0;

  Boid(PVector position, float heading, float max_speed, float max_rotational_speed, float acceleration, float rotational_acceleration)
  {
    this.kinematic = new KinematicMovement(position, heading, max_speed, max_rotational_speed);
    this.last_crumb = millis();
    this.max_acceleration = acceleration;
    this.max_rotational_acceleration = rotational_acceleration;
  }
  
  float distanceToNextPoint(int index) {
    int nextIndex = index+1;
    if (nextIndex >= points.size())
      return -1;
    else {
      PVector currentPoint = points.get(index);
      PVector nextPoint = points.get(nextIndex);
      return distanceBetween(currentPoint, nextPoint);
    }
  }
  
  boolean shouldChangeFinishingRadius(int currentIndex)
  {
    float result = distanceToNextPoint(currentIndex);
    if (result < 0 || result > defaultFinishRadius) return false;
    else return true;
  }

  //another helper function because i can't remember trigonometry that well
  void drawAngledLine(float radian, String tag)
  {
    int lineLength = 200;
    float y1 = sin(radian) * lineLength;
    float x1 = cos(radian) * lineLength;
    line(kinematic.position.x, kinematic.position.y, kinematic.position.x + x1, kinematic.position.y + y1);

    text(tag + " " + radian, kinematic.position.x + x1, kinematic.position.y + y1);
  }

  //helper function because i do not like writing this over and over --Victor
  int signOf(float n)
  {
    return (n > 0)? 1: -1;
  }

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

  boolean movingToLastPoint() {
    return (points==null || points.size() == 0) || (points != null && currentPointIndex == points.size()-1);
  }

  float angleToNextPoint;

  float getAngleBetween(PVector A, PVector B, PVector C) {
    PVector AB = PVector.sub(B, A);
    PVector BC = PVector.sub(C, B);
    float theta = PVector.angleBetween(AB, BC);
    println(theta * 52.97);
    return theta;
  }

  void getAngleToNextPoint() {
    angleToNextPoint = getAngleBetween(points.get(currentPointIndex), points.get(currentPointIndex + 1), points.get(currentPointIndex + 2));
  }

  float slopeOf(PVector A, PVector B)
  {
    return ((B.y-A.y)/(B.x-A.x));
  }

  float defaultFinishRadius = 20;
  float finishRadius = 20;

  void update(float dt)
  {
    text(kinematic.getHeading(), kinematic.position.x, kinematic.position.y + 20);

    if (points != null) {
      for (int i = 0; i<points.size(); i++)
      {
        circle(points.get(i).x, points.get(i).y, (i==currentPointIndex)? 20: 10);
        text(i, points.get(i).x + 10, points.get(i).y + 10);
      }
    }

    if (target != null)
    {
      circle(origin.x, origin.y, 10);
      circle(target.x, target.y, finishRadius);

      //grab direction of target
      float targetRadian = atan2(target.y - kinematic.position.y, target.x - kinematic.position.x);

      //find the shortest radian distance between target and you (that way you don't do a giant rotation and mess up your speed)
      //this value is between -pi and pi
      float shortestRadianDistance = shortestRadianBetween(targetRadian, kinematic.getHeading());
      boolean clockwise = shortestRadianDistance >= 0;

      float rotation = signOf(shortestRadianDistance) * max_rotational_acceleration;

      //first expression is self explanatory, second expressed will determine the slowdown marker
      //given acceleration, a destination angle, and a radian distance to a destination angle, you could calculate the radian angle at which you ought to slow down
      //therefore, if you're going too fast towards a target, we'll check if you've crossed this threshold yet.
      //if you've crossed it, slow down.
      boolean clockwiseAndApproaching = clockwise && shortestRadianBetween(kinematic.getHeading(), targetRadian - (kinematic.rotational_velocity/2)) > 0;
      boolean counterClockwiseAndApproaching = !clockwise && shortestRadianBetween(kinematic.getHeading(), targetRadian - (kinematic.rotational_velocity/2)) < 0;
      //println(kinematic.getHeading() + kinematic.rotational_velocity);
      //println(shortestRadianBetween(kinematic.getHeading(), targetRadian + kinematic.rotational_velocity));
      boolean approaching = clockwiseAndApproaching || counterClockwiseAndApproaching;

      //if you're coming close, then slow down!
      if (approaching)
      {
        rotation =  signOf(rotation) * -1;
      }

      drawAngledLine(kinematic.getHeading(), "heading");
      drawAngledLine(targetRadian, "target angle");
      //drawAngledLine(targetRadian - kinematic.rotational_velocity, "catching line");
      //drawAngledLine(targetRadian + kinematic.rotational_velocity, "catching line");

      text("Shortest Radian Distance: " + shortestRadianDistance + TWO_PI, kinematic.position.x + 50, kinematic.position.y + 20);
      text("Rotational Velocity: " + kinematic.rotational_velocity, kinematic.position.x + 50, kinematic.position.y + 35);
      text("Rotation: " + rotation, kinematic.position.x + 50, kinematic.position.y + 50);
      text("Clockwise: " + clockwise, kinematic.position.x + 50, kinematic.position.y + 65);
      text("Approaching: " + approaching, kinematic.position.x + 50, kinematic.position.y + 80);

      float targetToOrigin = distanceBetween(target, origin);
      float targetToBoid = distanceBetween(target, kinematic.position);
      
      speed = max_acceleration;
           
      float progress = targetToBoid / targetToOrigin;  //closer it is to 0, the closer the boid is to the target
      float velocityAmount = kinematic.speed/kinematic.max_speed; 

      float brakeMultiplier = ((min(HALF_PI, abs(angleToNextPoint)))/2);
      float slowdownThreshold = pow(velocityAmount * brakeMultiplier, 2);
      //circle(target.x,target.y, slowdownThreshold * targetToOrigin);
      if (progress < slowdownThreshold)
      {
        speed = -max_acceleration;
      }
      
      float speedDampen = 3;
      if (abs(kinematic.rotational_velocity) > 1 && kinematic.speed > 30) {
        speed -= (abs(shortestRadianDistance)/PI * speed) * 2 * speedDampen;
      }
      
      //it's pretty much done at this point
      if (targetToBoid < finishRadius) {
        println("Finished with point: " + currentPointIndex); //<>// //<>//
        if (points != null && (points.size() > 2 || currentPointIndex < points.size()-1))
        {
          if (currentPointIndex < points.size()-2)
          {
            getAngleToNextPoint();
          }

          if (!movingToLastPoint())
          {
            currentPointIndex++;
            seek(points.get(currentPointIndex));
            println("Moving to point: " + currentPointIndex);
          }
        }
      }

      if (movingToLastPoint())
      {
        if (progress < velocityAmount) {
          speed = -max_acceleration;
        }
      }


      kinematic.increaseSpeed(speed * progress, rotation * dt);
      //circle(kinematic.position.x, kinematic.position.y, kinematic.speed);
      //circle(target.x, target.y, kinematic.speed * (kinematic.speed / kinematic.max_speed));

      text("Positional Velocity: " + speed, kinematic.position.x + 50, kinematic.position.y + 95);
      text("Progress: " + progress, kinematic.position.x + 50, kinematic.position.y + 110);
      text("Velocity Amount: " + velocityAmount, kinematic.position.x + 50, kinematic.position.y +125);
      text("Target to Boid: " + targetToBoid, kinematic.position.x + 50, kinematic.position.y + 140);
      text("Angle to Next Point: " + angleToNextPoint, kinematic.position.x + 50, kinematic.position.y + 155);
      text("Kinematic Velocity: " + kinematic.speed, kinematic.position.x + 50, kinematic.position.y + 170);
      text("Braking Multiplier: " + brakeMultiplier, kinematic.position.x + 50, kinematic.position.y + 185);
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

  PVector origin;
  float speed;  // contains "max_acceleration * dt", mainly for brevity

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
    if (shouldChangeFinishingRadius(currentPointIndex))
      finishRadius = defaultFinishRadius / 2;
    else finishRadius = defaultFinishRadius;
    
    println("Origin: " + origin);
    println("Target: " + target);
  }

  void follow(ArrayList<PVector> waypoints)
  {
    // TODO: change to follow *all* waypoints
    points = waypoints;
    currentPointIndex = 0;
    if (points.size() > 2) angleToNextPoint = getAngleBetween(kinematic.position, points.get(0), points.get(1));
    seek(points.get(currentPointIndex));
  }
}
