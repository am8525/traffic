package behavior;

import core.Behavior;
import core.Car;
import core.World;
import processing.core.PVector;

/**
 * Braking behavior - apply maximum braking.
 */
public class Brake extends Behavior {

	/**
	 * Create a braking behavior - apply maximum braking force.
	 * 
	 * @param c
	 *          color to display steering vector (for debug mode)
	 */
	public Brake ( int c ) {
		super(c);
	}

	/**
	 * Get the brake steering force for the specified car.
	 * 
	 * @param car
	 *          the car
	 * @param world
	 *          the world containing the car
	 * @return the steering force for the specified car
	 */
	public PVector getSteeringForce ( Car car, World world ) {
		// maximum braking force
		PVector steering = PVector.mult(car.getVelocity(),-1);
		steering.setMag(car.getMaxBrake());

		world.debugVector(World.DEBUG_BEHAVIOR,car.getCenter(),steering,40,color_,2);

		return steering;
	}
}