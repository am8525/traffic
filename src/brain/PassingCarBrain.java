package brain;

import behavior.ChangeLanes;
import behavior.Follow;
import behavior.TrackLane;
import core.Behavior;
import core.Brain;
import core.Car;
import core.Signal;
import core.World;
import processing.core.PVector;

/**
 * A car which drives forward as fast as it can in its current lane, changing
 * lanes when it encounters a slower car (and braking if necessary to avoid
 * collisions).
 */
public class PassingCarBrain implements Brain {

	protected Behavior lanechange_; // if not null, current in the process of
	                                // changing lanes
	protected int target_; // target lane for current lane change; -1 = none

	public PassingCarBrain () {
		lanechange_ = null;
		target_ = -1;
	}

	public PVector getNetSteeringForce ( Car car, World world ) {
		car.setBraking(false);

		// deal with lane changing - if car isn't in the lane yet, make sure the
		// lane is still open (based on neighborhood, so the car may have blind
		// spots)
		if ( lanechange_ != null && car.getLane() != target_ ) {
			for ( Car neighbor : world.getNeighbors(car) ) {
				if ( neighbor.getLane() == target_ ) {
					// car in the way of lane change - abandon lane change
					lanechange_ = null;
					target_ = -1;
					car.setSignal(Signal.NONE);
					break;
				}
			}
		}
		// deal with lane changing - actually steer
		if ( lanechange_ != null ) {
			PVector steer = lanechange_.getSteeringForce(car,world);
			if ( car.getLane() != target_ ) { // changing lanes
				return steer;
			} else { // lane change complete
				car.setSignal(Signal.NONE);
				lanechange_ = null;
				target_ = -1;
			}
		}

		// not currently changing lanes - if there's a slow car in front, brake but
		// also initiate a lane change for the next time step
		Car ahead = world.getNextCarInLane(car.getRoad(),car.getFrontBumper());
		if ( ahead != null ) {
			PVector follow = (new Follow(ahead,world.getApplet().color(255,0,0)))
			    .getSteeringForce(car,world);
			if ( follow.mag() > 0 ) {
				// flip a coin to decide which lane to change to - left or right
				int carlane = car.getRoad().getLane(car.getCenter());
				Signal dir = (Math.random() < .5 ? Signal.LEFT : Signal.RIGHT);
				// handle lanes on the edge of the road
				if ( carlane == 0 ) {
					dir = Signal.RIGHT;
				} else if ( carlane == car.getRoad().getNumLanes() - 1 ) {
					dir = Signal.LEFT;
				}
				int target = (dir == Signal.LEFT ? carlane - 1 : carlane + 1);
				// is the lane open? based on neighborhood - the car may have blind
				// spots!
				for ( Car neighbor : world.getNeighbors(car) ) {
					if ( neighbor.getLane() == target ) {
						// car in the way of lane change; abandon lane change
						dir = Signal.NONE;
						target = -1;
						lanechange_ = null;
						break;
					}
				}
				// set up lane change if that's still the intent
				if ( dir != Signal.NONE ) {
					car.setSignal(dir);
					lanechange_ =
					    new ChangeLanes(world.getApplet().color(255,0,255),target);
					target_ = target;
				}
				// brake to avoid the current imminent collision
				car.setBraking(true);
				return follow;
			}
		}

		// not changing lanes or braking - drive forward in the current lane
		Behavior track = new TrackLane(world.getApplet().color(255,0,225));
		return track.getSteeringForce(car,world);
	}

}