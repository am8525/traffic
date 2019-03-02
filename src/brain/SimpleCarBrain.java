package brain;

import behavior.Follow;

import behavior.TrackLane;
import core.Behavior;
import core.Brain;
import core.Car;
import core.Signal;
import core.World;
import processing.core.PVector;

/**
 * A simple car which just tries to drive forward as fast as it can in its
 * current lane, but does brake as needed to avoid collisions.
 */
public class SimpleCarBrain implements Brain {

	public PVector getNetSteeringForce ( Car car, World world ) {
		car.setBraking(false);
		car.setSignal(Signal.NONE);

		// slow down if there's a slow car ahead
		Car ahead = world.getNextCarInLane(car.getRoad(),car.getFrontBumper());
		if ( ahead != null ) {
			Behavior follow = new Follow(ahead,world.getApplet().color(255,0,0));
			PVector steering = follow.getSteeringForce(car,world);
			if ( steering.mag() > 0 ) {
				car.setBraking(true);
				return steering;
			}
		}
		
		// otherwise drive forward in the current lane
		Behavior track = new TrackLane(world.getApplet().color(255,0,225));
		return track.getSteeringForce(car,world);
	}

}