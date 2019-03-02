package behavior;

import core.Behavior;
import core.Car;
import core.World;
import processing.core.PVector;

/**
 * Track lane behavior - car wants to drive at its desired speed in its lane.
 */
public class TrackLane extends Behavior {

	/**
	 * Create a behavior to drive in the car's lane.
	 * 
	 * @param c
	 *          color to display steering vector (for debug mode)
	 */
	public TrackLane ( int c ) {
		super(c);
	}

	/**
	 * Get the track lane steering force for the specified car.
	 * 
	 * @param car
	 *          the car
	 * @param world
	 *          the world containing the car
	 * @return the steering force for the specified car
	 */
	public PVector getSteeringForce ( Car car, World world ) {
		// target is along the center line of the current lane of the road but at no
		// steeper an angle than the desired lane-changing angle

		// intersect lane-changing angle with road centerline
		PVector p0 = car.getRoad().getOnCenterStripe(car.getCenter());
		PVector p = car.getRoad().getOrientation().normalize(null);
		PVector q0 = car.getCenter();
		PVector q1 =
		    PVector.fromAngle(car.getRoad().getOrientation().heading()
		        - Car.LANE_CHANGE_ANGLE),
		    q2 = PVector.fromAngle(car.getRoad().getOrientation().heading()
		        + Car.LANE_CHANGE_ANGLE);

		float t1 =
		    (q1.x * (q0.y - p0.y) + q1.y * (p0.x - q0.x))
		        / (p.y * q1.x - p.x * q1.y),
		    t2 = (q2.x * (q0.y - p0.y) + q2.y * (p0.x - q0.x))
		        / (p.y * q2.x - p.x * q2.y);

		// System.out.println(q1+" "+q2+" / "+t1+" "+t2+" /
		// "+car.getDesiredSpeed());

		PVector target = PVector
		    .add(p0,PVector.mult(p,Math.max(car.getMaxSpeed(),Math.max(t1,t2))));

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