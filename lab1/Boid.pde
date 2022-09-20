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

  void update(float dt)
  {
    text(kinematic.getHeading(), kinematic.position.x, kinematic.position.y + 20);
    
    if (target != null)
    { 
      circle(target.x, target.y, 10);
      
      //grab direction of target
      float targetRadian = atan2(target.y - kinematic.position.y, target.x - kinematic.position.x);
      
      //find the shortest radian distance between target and you (that way you don't do a giant rotation and mess up your speed)
      //this value is between -pi and pi
      float shortestRadianDistance = min(targetRadian - normalize_angle_left_right(kinematic.getHeading()), TWO_PI - targetRadian - normalize_angle_left_right(kinematic.getHeading()));
      boolean clockwise = shortestRadianDistance >= 0;
      
      float rotation = shortestRadianDistance/PI * max_rotational_acceleration;
      
      //first expression is self explanatory, second expressed will determine the slowdown marker
      //given acceleration, a destination angle, and a radian distance to a destination angle, you could calculate the radian angle at which you ought to slow down
      //therefore, if you're going too fast towards a target, we'll check if you've crossed this threshold yet. 
      //if you've crossed it, slow down.
      boolean clockwiseAndApproaching = clockwise && kinematic.getHeading() > targetRadian - kinematic.rotational_velocity;
      boolean counterClockwiseAndApproaching = !clockwise && normalize_angle_left_right(kinematic.getHeading()) < targetRadian - kinematic.rotational_velocity;
      
      if (clockwiseAndApproaching || counterClockwiseAndApproaching)
      {
         rotation = max_rotational_acceleration * signOf(rotation) * -1;
      }
      
      drawAngledLine(kinematic.getHeading(),  "heading");
      drawAngledLine(targetRadian, "target angle");
      //drawAngledLine(targetRadian + kinematic.rotational_velocity, "stop here for clockwise");
      //drawAngledLine(targetRadian - kinematic.rotational_velocity, "stop here for counter clockwise");
      
      text("Shortest Radian Distance: " + shortestRadianDistance + TWO_PI, kinematic.position.x + 50, kinematic.position.y + 20);
      text("Rotational Velocity: " + kinematic.rotational_velocity, kinematic.position.x + 50, kinematic.position.y + 35);
      text("Rotation: " + rotation, kinematic.position.x + 50, kinematic.position.y + 50);
      text("Clockwise: " + clockwise, kinematic.position.x + 50, kinematic.position.y + 65);
      
      kinematic.increaseSpeed(max_acceleration * dt, rotation * dt);
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
