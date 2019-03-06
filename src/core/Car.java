package core;

import processing.core.PApplet;
import processing.core.PVector;

/**
 * A car. The car is always oriented along the road it is traveling on
 * 
 * @author ssb
 */
public class Car {

	public static final float LANE_CHANGE_ANGLE = PApplet.radians(45);

	// ID to use for next car created
	protected static int nextid_ = 0;

	// car's ID
	protected int id_;

	// simple vehicle model parameters
	protected PVector position_; // center of car
	protected PVector velocity_;
	protected float mass_;
	protected float maxaccel_; // maximum forward acceleration that can be applied
	protected float maxspeed_; // max speed

	// additional driving properties of the car
	protected float maxbrake_; // maximum reverse acceleration that can be applied

	// where the car is driving
	protected Road road_;

	// current state of car's turn signal and brake lights
	protected Signal signal_;
	protected boolean braking_; // true if braking, false if not
	protected int target_; //target for lane change; -1 if none.

	// car's color and dimensions
	protected int color_;
	protected float length_;
	protected float width_;

	// parameters defining the neighborhood
	protected float neighborRadius_;
	protected float neighborAngle_;

	// action selection
	protected Brain brain_;

	protected World world_; // world the car belongs to

	/**
	 * Create a new car.
	 * 
	 * @param world
	 *          the world containing the car
	 * @param color
	 *          car's color
	 * @param width
	 *          car's width (in the usual sense - perpendicular to the direction
	 *          of travel)
	 * @param length
	 *          car's length (in the usual sense - parallel to the direction of
	 *          travel)
	 * @param mass
	 *          car's mass
	 * @param position
	 *          center of the car
	 * @param velocity
	 *          car's velocity
	 * @param maxaccel
	 *          car's maximum acceleration force
	 * @param maxbrake
	 *          car's maximum braking force
	 * @param maxspeed
	 *          car's maximum speed
	 * @param neighborRadius
	 *          radius of car's neighborhood
	 * @param neighborAngle
	 *          car's field of view
	 * @param brain
	 *          car's brain (for selecting/combining behaviors)
	 */
	public Car ( World world, int color, float width, float length, float mass,
	             PVector position, PVector velocity, float maxaccel,
	             float maxbrake, float maxspeed, float neighborRadius,
	             float neighborAngle, Brain brain ) {
		id_ = nextid_;
		nextid_++;

		world_ = world;

		color_ = color;
		length_ = length;
		width_ = width;
		mass_ = mass;

		position_ = position;
		velocity_ = velocity;

		maxaccel_ = maxaccel;
		maxbrake_ = maxbrake;
		maxspeed_ = maxspeed;

		neighborRadius_ = neighborRadius;
		neighborAngle_ = neighborAngle;

		road_ = null;
		signal_ = Signal.NONE;
		target_ = -1;
		braking_ = false;

		brain_ = brain;
	}

	/**
	 * Get the distance covered while slowing from current speed to target.
	 * 
	 * @param target
	 *          car's desired speed
	 * @return distance covered while braking, or 0 if current speed <= target
	 */
	public float getBrakingDist ( float target ) {
		return getBrakingDist(getSpeed(),target);
	}

	/**
	 * Get the distance covered while slowing from the specified speed to target.
	 * 
	 * @param speed
	 *          speed
	 * @param target
	 *          car's desired speed
	 * @return distance covered while braking, or 0 if speed <= target
	 */
	public float getBrakingDist ( float speed, float target ) {
		if ( speed <= target ) {
			return 0;
		} else {
			return (target * target - speed * speed) / (-2 * maxbrake_);
		}
	}

	/**
	 * Get the time needed to slow from current speed to target.
	 * 
	 * @param target
	 *          car's desired speed
	 * @return time needed to brake, or 0 if current speed <= target
	 */
	public float getBrakingTime ( float target ) {
		return getBrakingTime(getSpeed(),target);
	}

	/**
	 * Get the time needed to slow from the specified speed to target.
	 * 
	 * @param speed
	 *          speed
	 * @param target
	 *          car's desired speed
	 * @return time needed to brake, or 0 if speed <= target
	 */
	public float getBrakingTime ( float speed, float target ) {
		if ( speed <= target ) {
			return 0;
		} else {
			return (speed - target) / maxbrake_;
		}
	}

	/**
	 * Get the car's position. (center of the car)
	 * 
	 * @return car's current position (center of the car)
	 */
	public PVector getCenter () {
		return position_;
	}

	/**
	 * Get the car's position t time units in the future, assuming car continues
	 * at its current velocity. (center of the car)
	 * 
	 * @return car's position (center of the car)
	 */
	public PVector getCenter ( float t ) {
		return road_.getAlong(position_,t*getSpeed());
	}

	/**
	 * Get the car's color.
	 * 
	 * @return car's color
	 */
	public int getColor () {
		return color_;
	}

	/**
	 * Get the position of the center of the car's front bumper.
	 * 
	 * @return position of the center of the car's front bumper
	 */
	public PVector getFrontBumper () {
		PVector orientation = getOrientation();
		PVector dir = new PVector(orientation.x,orientation.y);
		dir.normalize();
		return PVector.add(position_,PVector.mult(dir,length_ / 2));
	}

	/**
	 * Get the position of the center of the car's front bumper t time units in
	 * the future, assuming car continues at its current velocity.
	 * 
	 * @return position of the center of the car's front bumper
	 */
	public PVector getFrontBumper ( float t ) {
		PVector orientation = getOrientation();
		PVector dir = new PVector(orientation.x,orientation.y);
		dir.normalize();
		return PVector.add(road_.getAlong(position_,t*getSpeed()),PVector.mult(dir,length_ / 2));	
	}

	/**
	 * Get the car's ID.
	 */
	public int getID () {
		return id_;
	}

	/**
	 * Get which lane the car is in. (getRoad() != null)
	 * 
	 * @return which lane the car is in
	 */
	public int getLane () {
		return road_.getLane(position_);
	}

	/**
	 * Get the length of the car.
	 * 
	 * @return car's length
	 */
	public float getLength () {
		return length_;
	}

	/**
	 * Get the car's max acceleration force.
	 * 
	 * @return car's max acceleration force
	 */
	public float getMaxAccel () {
		return maxaccel_;
	}

	/**
	 * Get the car's maximum braking power. (>= 0)
	 */
	public float getMaxBrake () {
		return maxbrake_;
	}

	/**
	 * Get the car's max speed.
	 * 
	 * @return car's max speed
	 */
	public float getMaxSpeed () {
		return maxspeed_;
	}

	/**
	 * Get the car's field of view.
	 * 
	 * @return car's field of view
	 */
	public float getNeighborAngle () {
		return neighborAngle_;
	}

	/**
	 * Get the radius of the car's field of view.
	 * 
	 * @return radius of car's field of view.
	 */
	public float getNeighborRadius () {
		return neighborRadius_;
	}

	/**
	 * Get the car's orientation. The car is oriented along the road it is driving
	 * on, or in the direction of its velocity if it isn't on a road.
	 * 
	 * @return the car's forward vector
	 */
	public PVector getOrientation () {
		return (road_ == null ? velocity_ : road_.getOrientation());
	}

	/**
	 * Get the position of the car's rear bumper. (center of the bumper)
	 * 
	 * @return position of the car's rear bumper
	 */
	public PVector getRearBumper () {
		PVector orientation = getOrientation();
		PVector dir = new PVector(orientation.x,orientation.y);
		dir.normalize();
		return PVector.add(position_,PVector.mult(dir,-length_ / 2));
	}

	/**
	 * Get the position of the center of the car's rear bumper t time units in the
	 * future, assuming car continues at its current velocity.
	 * 
	 * @return position of the center of the car's rear bumper
	 */
	public PVector getRearBumper ( float t ) {
		PVector orientation = getOrientation();
		PVector dir = new PVector(orientation.x,orientation.y);
		dir.normalize();
		return PVector.add(road_.getAlong(position_,t*getSpeed()),PVector.mult(dir,-length_ / 2));	
	}

	/**
	 * Get the road the car is driving on.
	 * 
	 * @return road the car is driving on
	 */
	public Road getRoad () {
		return road_;
	}

	/**
	 * Get the state of the car's turn signal.
	 * 
	 * @return state of the car's turn signal
	 */
	public Signal getSignal () {
		return signal_;
	}
	
	/**
	 * Get the int value of the lane we're changing into
	 * 
	 * @return int value of the lane we're going to
	 */
	public int getTargetLane() {
		return target_;
	}

	/**
	 * Get the car's current speed. (speed = magnitude of the velocity)
	 * 
	 * @return car's current speed
	 */
	public float getSpeed () {
		return velocity_.mag();
	}

	/**
	 * Get the car's velocity.
	 * 
	 * @return car's current velocity
	 */
	public PVector getVelocity () {
		return velocity_;
	}

	/**
	 * Is the car braking?
	 * 
	 * @return true if the car is braking, false if not
	 */
	public boolean isBraking () {
		return braking_;
	}

	/**
	 * Determine if 'other' is a neighbor of this car (i.e. is in this car's
	 * neighborhood)
	 * 
	 * @param other
	 *          other car
	 * @return true if 'other' is a neighbor of this car, false if 'other' is not
	 *         in the neighborhood or if 'other' is the same car (a car is not its
	 *         own neighbor)
	 */
	public boolean isNeighbor ( Car other ) {
		if ( other == this ) {
			return false;
		}

		float dist1 = PVector.dist(position_,other.getFrontBumper());
		float dist2 = PVector.dist(position_,other.getRearBumper());
		if ( dist1 > neighborRadius_ && dist2 > neighborRadius_ ) {
			return false;
		}

		float angle = PVector.angleBetween(getOrientation(),
		                                   PVector.sub(other.position_,position_));
		return PApplet.abs(angle) <= neighborAngle_;
	}

	/**
	 * Draw the car on the screen, using its current position and orientation.
	 */
	public void render ( PApplet parent ) {
		if ( world_.getDebug(World.DEBUG_CAR) ) {
			// draw the neighborhood
			parent.ellipseMode(PApplet.CENTER);
			parent.stroke(255);
			parent.fill(255,50);

			parent.pushMatrix();
			parent.translate(position_.x,position_.y);
			PVector orientation = getOrientation();
			parent.rotate(PApplet.atan2(orientation.y,orientation.x));
			parent.arc(0,0,neighborRadius_ * 2,neighborRadius_ * 2,-neighborAngle_,
			           neighborAngle_,PApplet.PIE);
			parent.popMatrix();
		}

		parent.stroke(0);
		parent.rectMode(PApplet.CORNER);
		parent.ellipseMode(PApplet.CENTER);

		parent.pushMatrix();
		parent.translate(position_.x,position_.y);
		PVector orientation = getOrientation();
		parent.rotate(PApplet.atan2(orientation.y,orientation.x));
		switch ( signal_ ) {
		case LEFT:
			parent.fill(255,0,0);
			parent.ellipse(length_ / 2 - 2,-width_ / 2 - 2,4,4);
			break;
		case RIGHT:
			parent.fill(255,0,0);
			parent.ellipse(length_ / 2 - 2,width_ / 2 + 2,4,4);
			break;
		case NONE:
			break;
		}
		parent.fill(color_);
		parent.rect(-length_ / 2,-width_ / 2,length_,width_);
		parent.fill(255);
		parent.text((id_ < 10 ? " " : "") + id_,-length_ / 2,-width_ / 2 - 3);
		if ( braking_ ) {
			parent.fill(255,0,0);
			parent.rect(-length_ / 2 - 2,-width_ / 2,2,width_);
		}
		parent.popMatrix();
	}

	/**
	 * Set whether the car is braking.
	 * 
	 * @param braking
	 *          true if the car is braking, false otherwise
	 */
	public void setBraking ( boolean braking ) {
		braking_ = braking;
	}

	/**
	 * Set the car's position.
	 * 
	 * @param position
	 *          car's new position
	 */
	public void setPosition ( PVector position ) {
		position_ = position;
	}

	/**
	 * Set the road the car is driving on.
	 * 
	 * @param road
	 *          the road
	 */
	public void setRoad ( Road road ) {
		road_ = road;
	}

	/**
	 * Set the state of the car's turn signal.
	 * 
	 * @param signal
	 *          new value for turn signal
	 */
	public void setSignal ( Signal signal ) {
		signal_ = signal;
	}
	
	/**
	 * Set the target lane of the car.
	 * 
	 * @param target
	 * 			new value for target_
	 */
	public void setTargetLane(int target) {
		target_ = target;
	}

	/**
	 * Move the car according to its behaviors. Also displays debugging info if
	 * debug mode is on.
	 */
	public void update () {
		// compute and apply the net steering force
		PVector steer = brain_.getNetSteeringForce(this,world_);
		PVector accel = PVector.div(steer,mass_);
		accel.limit(maxaccel_);

		// // is this accelerating or braking?
		// if ( accel.mag() == 0 || PVector.dot(accel,velocity_) >= 0 ) {
		// // accelerating
		// braking_ = false;
		// } else {
		// // braking
		// braking_ = true;
		// }
		//
		// // are we turning?
		// float ccw =
		// ccw(position_,PVector.add(position_,velocity_),
		// PVector.add(PVector.add(position_,velocity_),getOrientation()));
		// if ( ccw == 0 ) {
		// signal_ = Signal.NONE;
		// } else if ( ccw > 0 ) {
		// // turning left
		// signal_ = Signal.LEFT;
		// } else {
		// // turning right
		// signal_ = Signal.RIGHT;
		// }

		velocity_.add(accel);
		velocity_.limit(maxspeed_);

		position_.add(velocity_);
	}
}