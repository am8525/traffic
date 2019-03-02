package behavior;

import core.Behavior;
import core.Car;
import core.World;
import processing.core.PVector;

/**
 * Brakes as needed to avoid running into the specified car.
 */
public class Follow extends Behavior {

	private Car ahead_;

	/**
	 * Create a behavior to brake as needed to avoid running into the specified
	 * car.
	 * 
	 * @param ahead
	 *          car ahead to avoid
	 * @param c
	 *          color to display steering vector (for debug mode)
	 */
	public Follow ( Car ahead, int c ) {
		super(c);
		ahead_ = ahead;
	}

	/**
	 * Get the follow steering force for the specified car.
	 * 
	 * @param car
	 *          the car
	 * @param world
	 *          the world containing the car
	 * @return the steering force for the specified car
	 */
	public PVector getSteeringForce ( Car car, World world ) {
		PVector steering = new PVector(0,0);

		// brake to avoid car ahead

		// compute how long it takes this car to slow down, and how far ahead
		// the next car will be if that happens
		float braketime = car.getBrakingTime(ahead_.getSpeed());
		PVector aheadpos = ahead_.getRearBumper(braketime);

		// car will need to start braking brakedist before aheadpos
		float brakedist = car.getBrakingDist(ahead_.getSpeed()) + World.SPACING;
		float aheaddist = PVector.dist(car.getFrontBumper(),aheadpos);

		if ( aheaddist <= brakedist ) {
			steering = PVector.mult(car.getVelocity(),-1);
			steering.setMag(car.getMaxBrake());
		}

		world.debugVector(World.DEBUG_BEHAVIOR,car.getCenter(),steering,40,color_,2);

		return steering;
	}
}