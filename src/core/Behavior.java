package core;

import processing.core.PVector;

/**
 * A behavior that results in a steering force.
 */
public abstract class Behavior {

	protected int color_; // color to use for displaying debugging info

	/**
	 * Create a behavior.
	 * 
	 * @param c
	 *          color to use for the debugging display
	 */
	protected Behavior ( int c ) {
		color_ = c;
	}

	/**
	 * Get the color to use for the debugging display.
	 * 
	 * @return color to use for the debugging display
	 */
	public int getColor () {
		return color_;
	}

	/**
	 * Get the steering force for the specified car according to this behavior.
	 * 
	 * @param car
	 *          the car
	 * @param world
	 *          the world containing the car
	 * @return the steering force for the specified car
	 */
	public abstract PVector getSteeringForce ( Car car, World world );
}
