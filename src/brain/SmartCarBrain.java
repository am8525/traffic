package brain;

import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;

import behavior.Brake;
import behavior.ChangeLanes;
import behavior.Follow;
import behavior.TrackLane;
import core.Behavior;
import core.Brain;
import core.Car;
import core.Goal;
import core.Signal;
import core.State;
import core.World;
import pathfinding.CostNode;
import pathfinding.DynamicPathfinderGraph;
import pathfinding.RoadGraphNode;
import processing.core.PApplet;
import processing.core.PVector;

/**
 * A smart car, which has a goal and can navigate its way around other traffic.
 */
public class SmartCarBrain implements Brain {

	private Goal goal_; //where we're trying to get to. Used with heuristic evaluation.

	private DynamicPathfinderGraph graph_; //to be used with A*.
	private ArrayList<RoadGraphNode> path_; //used to track our path.

	private boolean pathOn_; //flag for path debugging

	private int target_; //target for lane change; -1 if none.

	private Behavior lanechange_; //null if not currently changing lanes
	private State state_; //current state of the smart car

	private long start_; //start time of the pathfinding


	/*
	 * States:
	 * 	-Braking
	 * 
	 *  -ChangingLanes
	 *  
	 *  -Fwd
	 */

	private PriorityQueue<CostNode> pq_;  //PQ for A*

	public SmartCarBrain () {
		pathOn_ = false;
		goal_ = null;
		target_ = -1;
		start_ = -1;
		pq_ = new PriorityQueue<CostNode>();
		state_ = State.Fwd_MaxSpeed;
	}

	public PVector getNetSteeringForce ( Car car, World world ) {

		if (start_ == -1) { //start the pathfinding
			start_ = System.nanoTime();
		}

		//generate next nodes if not in base state
		if (state_ != State.Fwd_MaxSpeed) {
			long elapsed = System.nanoTime() - getStartTime();
			for (RoadGraphNode node: graph_.getNextLocations(new RoadGraphNode(car.getRoad(),
			                                                                   car.getCenter(),System.nanoTime()))) {
				
				long etaNode = (long) ((PVector.sub(car.getCenter(),node.getPosition()).mag()) 
						/ car.getMaxSpeed());		
				long etaGoal = (long) ((PVector.sub(goal_.getPoint(),node.getPosition()).mag()) 
						/ car.getMaxSpeed());			
				long total = elapsed + etaNode + etaGoal;
				pq_.add(new CostNode(node,total));
			}	
		}

		Car ahead = null;
		Behavior track = null;
		PVector abort = null;

		switch (state_) {
		/*
		 * in this state, we transition to Braking if Follow returns a 
		 * non-zero magnitude
		 */
		case Fwd_MaxSpeed:
			System.out.println("fwd");
			ahead = world.getNextCarInLane(car.getRoad(),car.getFrontBumper());
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
						state_ = State.ChangingLanes;
					}
					else {
						state_ = State.Braking;
					}
					// brake to avoid the current imminent collision
					car.setBraking(true);					
					return follow;
				}
			}
			// not changing lanes or braking - drive forward in the current lane
			track = new TrackLane(world.getApplet().color(255,0,225));
			return track.getSteeringForce(car,world);

		case Braking:
			System.out.println("braking");
			ahead = world.getNextCarInLane(car.getRoad(),car.getFrontBumper());
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
						state_ = State.ChangingLanes;
					}
					else {
						state_ = State.Braking;
					}
					// brake to avoid the current imminent collision
					car.setBraking(true);					
					return follow;
				}
			}
			car.setBraking(false);
			state_ = State.Fwd_MaxSpeed;
			// not changing lanes or braking - drive forward in the current lane
			track = new TrackLane(world.getApplet().color(255,0,225));
			return track.getSteeringForce(car,world);


		case ChangingLanes:
			car.setBraking(false);
			System.out.println("changing lanes");
			if (car.getLane() != target_ && lanechange_ != null) {
				for (Car other : world.getNeighbors(car)) {
					if (other.getLane() == target_) {

						target_ = -1;
						car.setSignal(Signal.NONE);
						abort = new PVector(lanechange_.getSteeringForce(car,world).x,
						                    -lanechange_.getSteeringForce(car,world).y);
						state_ = State.AbortingLaneChange;
						lanechange_ = null;
						return abort;
					}
				}
			}
			if (lanechange_ != null) {
				PVector steer = lanechange_.getSteeringForce(car,world);
				if ( car.getLane() != target_ ) { // changing lanes
					return steer;
				} else { // lane change complete
					car.setSignal(Signal.NONE);
					lanechange_ = null;
					target_ = -1;
					state_ = State.Fwd_MaxSpeed;
				}			
			}
			// not changing lanes or braking - drive forward in the current lane
			track = new TrackLane(world.getApplet().color(255,0,225));
			return track.getSteeringForce(car,world);
		case AbortingLaneChange:
			System.out.println("aborting lane change");
			track = new TrackLane(world.getApplet().color(255,0,225));
			return track.getSteeringForce(car,world);
		}
		
		

		return new PVector(1,0);
	}

	/**
	 * Setter for the goal.
	 * Note that this value is set in Traffic's setup() method.
	 * 
	 * @param goal 
	 * 			Goal of the smart car.
	 */
	public void setGoal ( Goal goal ) {
		goal_ = goal;
	}

	/**
	 * Used to initialized the DynamicPathfinderGraph.
	 * @param world
	 * 			Our world
	 * @param car
	 * 			The smart car
	 */
	public void initGraph(World world, Car car) {
		graph_ = new DynamicPathfinderGraph(car,world,goal_);
	}

	/**
	 * Get the int value of the lane we're changing into
	 * 
	 * @return int value of the lane we're going to
	 */
	public int getTargetLane() {
		return target_;
	}

	/**
	 * Set the target lane of the car.
	 * 
	 * @param target
	 * 			new value for target_
	 */
	public void setTargetLane(int target) {
		target_ = target;
	}

	/**
	 * Used to set the start time of a simulation.
	 * @param start
	 * 			start time, long
	 */
	public void setStartTime(long start) {
		start_ = start;
	}

	/**
	 * Returns the simulation start time.
	 * @return
	 * 		start time of the simulation.
	 */
	public long getStartTime() {
		return start_;
	}

	private void debugPath ( List<RoadGraphNode> path, World world ) {

		if ( !world.getDebug(World.DEBUG_GRAPH) ) { return; }

		PApplet applet = world.getApplet();

		applet.ellipseMode(PApplet.CENTER);
		for ( int i = 0 ; i < path.size() - 1 ; i++ ) {
			// System.out.println("drawing path "+path_.get(i));
			PVector p1 = path.get(i).getPosition(),
					p2 = path.get(i + 1).getPosition();
			applet.stroke(0,255,255);
			applet.fill(0,255,255);
			applet.line(p1.x,p1.y,p2.x,p2.y);
			applet.ellipse(p2.x,p2.y,10,10);
		}
	}

}
