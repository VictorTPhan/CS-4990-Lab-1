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

  void update(float dt)
  {
    if (target != null)
    {
      //get the angle of the target in radians
      //since atan returns a range from -pi to pi, and our boid has a heading range of 0 to 2pi,
      //we add pi to targetRotation.
      float targetRotation = atan2(kinematic.position.y - target.y, kinematic.position.x - target.x) + PI;
      
      //the amount of radians required for this boid to turn to the target radian.
      float directRotation = targetRotation - kinematic.getHeading();
      float counter = TWO_PI - abs(directRotation);

      //if our targetRotation is to the right of our boid, we turn clockwise.
      //if it is to the left, we turn counterclockwise.
      boolean clockwise = false;
      if (targetRotation > kinematic.getHeading()) {
        clockwise = true;
      } else {
        clockwise = false;
      }

      //one issue that arises from our previous calculation is that if our
      //target radian is 1, and our boid is at radian 5, then since the target < heading,
      //the boid turns counterclockwise, even though the faster route would have been clockwise.
      //we can correct this behavior by checking if there exists a shorter path to turn.
      if (counter < abs(directRotation)) {
        directRotation = counter;
        clockwise = !clockwise;
      }

      
      boolean turningLeft = kinematic.rotational_velocity < 0;
      boolean targetOnLeft = targetRotation < kinematic.getHeading();
      boolean approachingTarget = (turningLeft && targetOnLeft) || (!turningLeft && !targetOnLeft);
      boolean correcting = !approachingTarget;

      //TEMPORARY
      //rotation is the amount of acceleration that we add every frame
      //rotation is expressed as a value from 0 to 1 representing how much speed is required to 
      //rotate to a given position. If the boid needs to turn pi degrees, it is 1, or 100% speed.
      float rotation = abs(directRotation)/HALF_PI;
      rotation *= max_rotational_acceleration; //combine multiplier and max acceleration
      
      /*----------------
      The issue with the code above is that the boid will always overshoot its target.
      If for example it starts moving at pi radians to a target 0 radians,
      in the first frame it will accelerate at max acceleration, then the next frame will 
      accelerate by a slightly smaller amount, and so on, which causes a build up of too much speed
      
      One attempted solution was to have a range around the target radian where the boid would
      have to slow down (apply negative acceleration), but another issue arises where turning
      the boid at too sharp of an angle would make it deccelerate too much and go backwards.
      
      Also note that the maximum decceleration amount cannot exceed -max_rotational_acceleration.
      We must:
      A. calculate a decceleration amount every frame given the current speed, current acceleration,
      and distance to the target radian (probable but it needs a lot of math)
      B. modify seek() so that every time it switches to a new target, it will calculate appropriate
      checkpoints along its radians. Every frame this boid will calculate whether or not it has passed
      a checkpoint, and apply an amount of acceleration/decceleration depending on which checkpoint it's
      at.
      *///----------------

      //rotation goes the other way if we're going counterclockwise
      if (!clockwise) {
        rotation *= -1;
      }

      //TEMPORARY (ALSO BAD CODE)
      //speed must be expressed as a relationship between the distance between the boid and target
      //more braking power must be used the closer it gets to the target
      float distanceToTarget = dist(target.x, target.y, kinematic.position.x, kinematic.position.y);
      float speed = 0;
      if (distanceToTarget > 250) {
        speed = kinematic.max_speed;
      } else {
        speed = kinematic.speed * -0.25f;
      }

      //2 parameters: positional velocity and rotational velocity
      kinematic.increaseSpeed(0 * dt, rotation * dt);
      //kinematic.increaseSpeed(0 * dt, 0 * dt);

      text("Rotational Multiplier: " + rotation, kinematic.position.x + 20, kinematic.position.y + 0);
      text("Going clockwise: " + clockwise, kinematic.position.x + 20, kinematic.position.y + 15);
      text("Rotational Velocity: " + kinematic.rotational_velocity, kinematic.position.x + 20, kinematic.position.y + 30);
      text("Turn Amount: " + abs(directRotation), kinematic.position.x + 20, kinematic.position.y + 60);
      text("Turning Left: " + turningLeft, kinematic.position.x + 20, kinematic.position.y + 75);
      text("Target On Left: " + targetOnLeft, kinematic.position.x + 20, kinematic.position.y + 90);
      text("Correcting Course: " + correcting, kinematic.position.x + 200, kinematic.position.y + 0);
      text("Approaching Target: " + approachingTarget, kinematic.position.x + 200, kinematic.position.y + 15);

      //how fast should we move?

      //arrival
      //if we get close, how do we slow down?
      //have a tolerance for reaching the target
      //handle overshoot behavior (break gradually!)

      //if you reach your target, you need to go to the next one too
      //some turns shouldn't slow down much, while some tighter angles require a good amount of slowdown
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
  }

  void follow(ArrayList<PVector> waypoints)
  {
    // TODO: change to follow *all* waypoints
    this.target = waypoints.get(0);
  }
}
