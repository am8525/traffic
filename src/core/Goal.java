package core;

import processing.core.PVector;

/**
 * A goal position for a car.
 */
public class Goal {

	private PVector p_; // the position
	private float radius_; // within the radius is considered achieving the goal

	/**
	 * Create a goal.
	 * 
	 * @param p
	 *          position of the goal
	 * @param radius
	 *          tolerance for achieving the goal - anywhere within the radius of p
	 *          is success
	 */
	public Goal ( PVector p, float radius ) {
		super();
		p_ = p;
		radius_ = radius;
	}

	/**
	 * Get the goal's point.
	 * 
	 * @return the point
	 */
	public PVector getPoint () {
		return p_;
	}

	/**
	 * Get the radius.
	 * 
	 * @return the radius of the goal
	 */
	public float getRadius () {
		return radius_;
	}

	/**
	 * Return true if the specified location satisfies this goal.
	 * 
	 * @param p
	 *          point
	 * @return true if p satisfies the goal, false if not
	 */
	public boolean matches ( PVector p ) {
		return PVector.dist(p,p_) <= radius_;
	}

	@Override
	public String toString () {
		return "Goal [p_=" + p_ + ", radius_=" + radius_ + "]";
	}

}
