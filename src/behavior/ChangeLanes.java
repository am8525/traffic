package behavior;

import core.Behavior;
import core.Car;
import core.Road;
import core.World;
import processing.core.PVector;

/**
 * Change lanes.
 */
public class ChangeLanes extends Behavior {

	private int lane_; // lane to change to

	/**
	 * Create a behavior to change lanes.
	 * 
	 * @param c
	 *          color to display steering vector (for debug mode)
	 * @param lane
	 *          the lane to change to
	 */
	public ChangeLanes ( int c, int lane ) {
		super(c);
		lane_ = lane;
	}

	/**
	 * Get the lane changing steering force for the specified car.
	 * 
	 * @param car
	 *          the car
	 * @param world
	 *          the world containing the car
	 * @return the steering force for the specified car
	 */
	public PVector getSteeringForce ( Car car, World world ) {
		// target is along the center line of the desired lane but at no steeper an
		// angle than the desired lane-changing angle

		Road road = car.getRoad();
		PVector target =
		    road.getLaneChangeTarget(car.getCenter(),lane_,Car.LANE_CHANGE_ANGLE);

		world.debugPoint(World.DEBUG_BEHAVIOR,target,color_,4);

		// seek the target
		PVector desired = PVector.sub(target,car.getCenter());
		desired.setMag(car.getMaxSpeed());
		PVector steering = PVector.sub(desired,car.getVelocity());
		steering.limit(car.getMaxAccel());

		world.debugVector(World.DEBUG_BEHAVIOR,car.getCenter(),steering,40,color_,2);

		return steering;
	}
}