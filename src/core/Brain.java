package core;

import processing.core.PVector;

/**
 * A car brain takes in information from the environment and produces an action
 * in the form of a net steering force.
 */
public interface Brain {

	/**
	 * Get the net steering force for the specified car.
	 * 
	 * @param car
	 *          the car
	 * @param world
	 *          the world containing the car
	 * @return net steering force for the specified car
	 */
	public PVector getNetSteeringForce ( Car car, World world );
}