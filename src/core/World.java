package core;

import java.time.Clock;
import java.util.ArrayList;
import java.util.List;

import processing.core.PApplet;
import processing.core.PVector;

/**
 * The world.
 */
public class World {

	/**
	 * Desired minimum spacing between cars.
	 */
	public static final float SPACING = 20;

	private PApplet applet_;

	// cars and roads
	private List<Car> cars_;
	private List<Road> roads_;
	
	//clock and time tracking vars.
//	private Clock clock_;
//	private long start_; //start time

	
	public static final int DEBUG_NONE = 0, DEBUG_CAR = 4, DEBUG_GRAPHGEN = 1,
	    DEBUG_GRAPH = 2, DEBUG_BEHAVIOR = 8;
	private int debug_; // debug status

	/**
	 * Create an empty world.
	 * 
	 * @param applet
	 *          parent applet (for drawing)
	 */
	public World ( PApplet applet ) {
		applet_ = applet;

		cars_ = new ArrayList<Car>();
		roads_ = new ArrayList<Road>();
		
		//clock_ = Clock.systemDefaultZone();
		//start_ = -1;
		
		debug_ = DEBUG_NONE;
	}

	/**
	 * Add a car to the world.
	 * 
	 * @param car
	 *          to add
	 */
	public void addCar ( Car car ) {
		cars_.add(car);
	}

	/**
	 * Add a road to the world.
	 * 
	 * @param road
	 *          to add
	 */
	public void addRoad ( Road road ) {
		roads_.add(road);
	}

	/**
	 * Get all the cars.
	 */
	public Iterable<Car> getCars () {
		return cars_;
	}

	/**
	 * Get all of the cars on the specified road with any part of the car in the
	 * specified interval. There are no guarantees on the order in which the cars
	 * appear in the list.
	 * 
	 * @param road
	 *          the road
	 * @param p
	 *          a point in the interval
	 * @param startoffset
	 *          offset from p to the start of the interval (<0 if the start is
	 *          before p along the road)
	 * @param endoffset
	 *          offset from p to the end of the interval (<0 if the end is before
	 *          p along the road)
	 * @return the cars in the interval - in the same lane as p, and within the
	 *         range p+startoffset and p+endoffset
	 */
	public List<Car> getCarsInInterval ( Road road, PVector p, float startoffset,
	                                     float endoffset ) {
		return getCarsInInterval(road,p,startoffset,endoffset,0);
	}

	/**
	 * Get all of the cars on the specified road with any part of the car in the
	 * specified interval at time t in the future. Assumes cars continue moving at
	 * their current velocities.There are no guarantees on the order in which the
	 * cars appear in the list.
	 * 
	 * @param road
	 *          the road
	 * @param p
	 *          a point in the interval
	 * @param startoffset
	 *          offset from p to the start of the interval (<0 if the start is
	 *          before p along the road)
	 * @param endoffset
	 *          offset from p to the end of the interval (<0 if the end is before
	 *          p along the road)
	 * @param t
	 *          time
	 * @return the cars in the interval - in the same lane as p, and within the
	 *         range p+startoffset and p+endoffset
	 */
	public List<Car> getCarsInInterval ( Road road, PVector p, float startoffset,
	                                     float endoffset, float t ) {
		if ( startoffset > endoffset ) {
			throw new IllegalArgumentException("start must come before end along the road");
		}

		int lane = road.getLane(p);

		if ( lane < 0 || lane >= road.getNumLanes() ) {
			throw new IllegalArgumentException("invalid lane; got " + lane);
		}

		PVector start = road.getAlong(p,startoffset),
		    end = road.getAlong(p,endoffset);

		List<Car> list = new ArrayList<Car>();
		for ( Car car : cars_ ) {
			if ( car.getRoad() != road ) { // not on this road
				continue;
			}
			PVector front = car.getFrontBumper(t);
			PVector rear = car.getRearBumper(t);
			if ( road.getLane(front) != lane ) {
				continue;
			}
			if ( road.compareTo(rear,end) <= 0 && road.compareTo(start,front) <= 0 ) {
				list.add(car);
			}
		}
		return list;
	}

	/**
	 * Get the neighbors of the specified car.
	 * 
	 * @param car
	 *          the car
	 * @return cars in the neighborhood of the specified car
	 */
	public List<Car> getNeighbors ( Car car ) {
		List<Car> neighbors = new ArrayList<Car>();
		for ( Car other : cars_ ) {
			if ( car.isNeighbor(other) ) {
				neighbors.add(other);
			}
		}
		return neighbors;
	}

	/**
	 * Get the first car in the lane whose rear bumper is at or ahead of p.
	 * 
	 * @param p
	 *          the position
	 * @return first car in the lane whose rear bumper is at or ahead of p, or
	 *         null if there are no such cars
	 */
	public Car getNextCarInLane ( Road road, PVector p ) {
		Car ahead = null;

		int lane = road.getLane(p);
		for ( Car car : cars_ ) {
			// not on this road or not in this lane or not ahead of this car
			if ( car.getRoad() != road
			    || car.getRoad().getLane(car.getFrontBumper()) != lane
			    || road.compareTo(car.getRearBumper(),p) < 0 ) {
				continue;
			}
			if ( ahead == null
			    || road.compareTo(car.getRearBumper(),ahead.getRearBumper()) < 0 ) {
				ahead = car;
			}
		}
		return ahead;
	}

	/**
	 * Get the parent applet, for Processing API operations.
	 * 
	 * @return parent
	 */
	public PApplet getApplet () {
		return applet_;
	}

	/**
	 * Get the roads in the world.
	 * 
	 * @return all the roads
	 */
	public Iterable<Road> getRoads () {
		return roads_;
	}

	/**
	 * Determine if the specified interval (in the same lane as p, and within the
	 * range p+startoffset and p+endoffset) is unoccupied.
	 * 
	 * @param road
	 *          the road
	 * @param p
	 *          a point in the interval
	 * @param startoffset
	 *          offset from p to the start of the interval (<0 if the start is
	 *          before p along the road)
	 * @param endoffset
	 *          offset from p to the end of the interval (<0 if the end is before
	 *          p along the road)
	 * @return true if the interval is empty (no car completely or partially in
	 *         it), false otherwise
	 */
	public boolean isIntervalVacant ( Road road, PVector p, float startoffset,
	                                  float endoffset ) {
		return isIntervalVacant(road,p,startoffset,endoffset,0);
	}

	/**
	 * Determine if the specified interval (in the same lane as p, and within the
	 * range p+startoffset and p+endoffset) is expected to be unoccupied at time
	 * t. Assumes cars continue moving at their current velocities.
	 * 
	 * @param road
	 *          the road
	 * @param p
	 *          a point in the interval
	 * @param startoffset
	 *          offset from p to the start of the interval (<0 if the start is
	 *          before p along the road)
	 * @param endoffset
	 *          offset from p to the end of the interval (<0 if the end is before
	 *          p along the road)
	 * @param t
	 *          time
	 * @return true if the interval is empty (no car completely or partially in
	 *         it), false otherwise
	 */
	public boolean isIntervalVacant ( Road road, PVector p, float startoffset,
	                                  float endoffset, float t ) {

		if ( startoffset > endoffset ) {
			throw new IllegalArgumentException("start must come before end along the road");
		}
		if ( t < 0 ) {
			throw new IllegalArgumentException("time cannot be in the past (t >= 0)");
		}

		int lane = road.getLane(p);

		if ( lane < 0 || lane >= road.getNumLanes() ) {
			throw new IllegalArgumentException("invalid lane; got " + lane);
		}

		PVector start = road.getAlong(p,startoffset),
		    end = road.getAlong(p,endoffset);

		// System.out.println("\t"+start+" "+end);
		for ( Car car : cars_ ) {
			if ( car.getRoad() != road ) { // not on this road
				continue;
			}
			PVector front = car.getFrontBumper(t);
			PVector rear = car.getRearBumper(t);
			// System.out.println("check interval: "+rear+" "+front+" / "+start+"
			// "+end);
			if ( road.getLane(front) != lane ) {
				// System.out.println("\twrong lane");
				continue;
			}
			if ( road.compareTo(rear,end) <= 0 && road.compareTo(start,front) <= 0 ) {
				return false;
			}
		}
		return true;
	}

	/**
	 * Draw the world.
	 */
	public void render () {
		for ( Road road : roads_ ) {
			road.render(applet_);
		}
		for ( Car car : cars_ ) {
			// System.out.println("rendering car "+car.getID()+"
			// "+car.getFrontBumper());
			car.render(applet_);
		}
	}

		/**
	 * Advance the simulation one step.
	 */
	public void update () {
		// System.out.println("update");
		for ( Car car : cars_ ) {
			// System.out.println("updating "+car.getID());
			car.update();

			// wrap if the car moves past the end of the road
			Road road = car.getRoad();
			int lane = road.getLane(car.getCenter());
			if ( road.compareTo(road.getEnd(lane),car.getRearBumper()) <= 0 ) {
				car.setPosition(road.getAlong(car.getFrontBumper(),-road.getLength()));
			}
		}
	}

	/**
	 * Get the debug status.
	 * 
	 * @return true if specified debug flag is set, false otherwise
	 */
	public boolean getDebug ( int flag ) {
		return (debug_ & flag) > 0;
	}

	/**
	 * Set the debug status.
	 * 
	 * @param flag
	 *          to set
	 * @param on
	 *          turn flag on if true, false if not
	 */
	public void setDebug ( int flag, boolean on ) {
		if ( on ) {
			debug_ |= flag;
		} else {
			debug_ &= ~flag;
		}
	}

	public void debugVector ( int flag, PVector pos, PVector v, float scale,
	                          int color, float weight ) {
		if ( !getDebug(flag) ) {
			return;
		}

		PVector n = PVector.mult(v,scale);

		applet_.stroke(color);
		applet_.strokeWeight(weight);
		applet_.line(pos.x,pos.y,pos.x + n.x,pos.y + n.y);

		applet_.strokeWeight(1);
	}

	public void debugLine ( int flag, PVector pos1, PVector pos2, int color,
	                        float weight ) {
		if ( !getDebug(flag) ) {
			return;
		}

		applet_.stroke(color);
		applet_.strokeWeight(weight);
		applet_.line(pos1.x,pos1.y,pos2.x,pos2.y);

		applet_.strokeWeight(1);
	}

	public void debugPoint ( int flag, PVector pos, int color, float size ) {
		if ( !getDebug(flag) ) {
			return;
		}

		applet_.stroke(color);
		applet_.fill(color);
		applet_.ellipseMode(PApplet.CENTER);
		applet_.ellipse(pos.x,pos.y,size,size);
	}

	public void debugCircle ( int flag, PVector pos, float radius, int color ) {
		if ( !getDebug(flag) ) {
			return;
		}

		applet_.stroke(color);
		applet_.noFill();
		applet_.ellipseMode(PApplet.CENTER);
		applet_.ellipse(pos.x,pos.y,radius * 2,radius * 2);
	}
	
//	/**
//	 * Gets the current time of our simulation clock.
//	 * @return
//	 * 		current time as a long.
//	 */
//	public long getCurrentTimeMS() {
//		return clock_.millis();
//	}
//	
//	/**
//	 * Used to set the start time of a simulation.
//	 * @param start
//	 * 			start time, long
//	 */
//	public void setStartTime(long start) {
//		start_ = start;
//	}
//	
//	/**
//	 * Returns the simulation start time.
//	 * @return
//	 * 		start time of the simulation.
//	 */
//	public long getStartTime() {
//		return start_;
//	}

}
