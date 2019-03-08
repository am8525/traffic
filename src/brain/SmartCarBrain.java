package brain;

import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Scanner;

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

	private Goal goal_; // where we're trying to get to. Used with heuristic
	// evaluation.

	private DynamicPathfinderGraph graph_; //to be used with A*.

	private ArrayList<CostNode> past_; //used to track where we've been.

	private ArrayList<CostNode> path_; // used to track our current path.

	private int target_; // target for lane change; -1 if none.

	private Behavior lanechange_; // null if not currently changing lanes
	private State state_; // current state of the smart car

	private float start_; //start time of the pathfinding
	/*
	 * States: -Braking -ChangingLanes -Fwd
	 */

	private PriorityQueue<CostNode> pq_; // PQ for A*

	/**
	 * Default constructor.
	 */
	public SmartCarBrain () {
		goal_ = null;
		target_ = -1;
		start_ = -1;
		pq_ = new PriorityQueue<CostNode>();
		past_ = new ArrayList<CostNode>();
		state_ = State.Fwd_MaxSpeed;
		path_=new ArrayList<CostNode>();
	}

	/**
	 * This method calculates the steering force determined 
	 * by the current conditions of the smart car.
	 * 
	 * @param car
	 * 		smart car
	 * 
	 * @param world 
	 * 		our world
	 */
	public PVector getNetSteeringForce ( Car car, World world ) {

		if ( start_ == -1 ) { // start the pathfinding, and generate initial graph.
			start_ = System.nanoTime();			

			//adding our start position.
			past_.add(new CostNode(new RoadGraphNode(car.getRoad(),car.getRoad().getStart(2),0),null,0.0f));
			pq_.clear();
			generateGraph(car,new RoadGraphNode(car.getRoad(),car.getCenter(),0));
		}

		this.debugPath(path_,world);

		Car ahead = null;
		Behavior track = null;
		PVector abort = null;


		//state begins in 'Fwd_MaxSpeed'
		switch ( state_ ) {
		/*
		 * 
		 * Another thing we have to think about is our path.
		 * If our position equals path_.get(0):
		 * 		remove that node from path,
		 * 		add it to past,
		 * 		and re-route to the next node.
		 */
		case Fwd_MaxSpeed:

			/*
			 * if we've reached the goal, we are done
			 */
			if (graph_.goalReached(new RoadGraphNode(car.getRoad(),car.getCenter(),0))) {

				System.out.println("past size: "+past_.size());
				for (int i = 0; i < past_.size(); i++) {
					System.out.println("node "+i+"... "+past_.get(i).getGraphNode().getPosition().x+", "+past_.get(i).getGraphNode().getPosition().y);
				}
				this.debugPath(past_,world);
				System.out.println("Goal reached in "+ ((System.nanoTime() - start_) / (Math.pow(10,9)) ));	
				System.out.println("Press enter in console to exit.");
				Scanner in = new Scanner(System.in);
				in.nextLine();

				System.out.println("Goodbye!");
				System.exit(0);

			}

			ahead = world.getNextCarInLane(car.getRoad(),car.getFrontBumper());
			PVector follow = null;
			//if there is a car ahead of us
			if ( ahead != null ) {
				follow = (new Follow(ahead,world.getApplet().color(255,0,0)))
						.getSteeringForce(car,world);

				//enters here if car ahead is within braking distance. Ensures a state change.
				if ( follow.mag() > 0 ) {

					//regenerate graph; looking for a new path. Note that we don't return anything yet.
					generateGraph(car,new RoadGraphNode(car.getRoad(),car.getCenter(),0));
					car.setBraking(true);
					state_ = State.Braking;
				}
			}

			//if we have arrived at our next node in the path, remove it from path, add to past, and re-route to our next node.
			if (PVector.dist(car.getCenter(),path_.get(path_.size() - 1).getGraphNode().getPosition()) < 2.5f) {

				CostNode old = path_.remove(path_.size() - 1);
				this.addPastNode(old);

				//consider changing lanes, only if we're not already braking.
				if (state_ != State.Braking) {
					//re-routing to our next node.
					int nextLane = -1;

					if (path_.size() > 0) {
						nextLane = car.getRoad().getLane(path_.get(path_.size() - 1).getGraphNode().getPosition());
					}
					//if the path is empty, we would like to re-generate it.
					else {
						this.generateGraph(car,new RoadGraphNode(car.getRoad(),car.getCenter(),0));
					}

					//if our next node is in another lane, get to that lane.
					if (nextLane != car.getLane()) {
						state_ = State.ChangingLanes;
						target_ = nextLane;
						lanechange_ = new ChangeLanes(123,target_);
						Signal dir = nextLane > car.getLane() ? Signal.RIGHT : Signal.LEFT;		
						car.setSignal(dir);
						return lanechange_.getSteeringForce(car,world);
					}
				}
			}
			if (state_ == State.Braking && follow != null) {
				return follow;
			}
			//We are safe to drive straight away, for now.
			else {
				track = new TrackLane(world.getApplet().color(255,0,225));
				return track.getSteeringForce(car,world);
			}

			/*
			 * when entering this state, we have just regenerated our graph.
			 * our next node should be in a different lane.
			 */
		case Braking:

			ahead = world.getNextCarInLane(car.getRoad(),car.getFrontBumper());
			follow = null;
			float distToCarAhead = 0.0f;

			//would be null if no car ahead
			if (ahead != null) {
				follow = (new Follow(ahead,world.getApplet().color(255,0,0)))
						.getSteeringForce(car,world);
				distToCarAhead = PVector.dist(ahead.getCenter(),car.getCenter());
			}

			/*
			 * if the distance to car ahead is less than twice our neighbor radius,
			 * we try changing lanes. else, go forward.
			 */
			if ( distToCarAhead < 2*car.getNeighborRadius() && follow != null) {

				float braketime = car.getBrakingTime(ahead.getSpeed());
				PVector aheadpos = ahead.getRearBumper(braketime);
				PVector steering = null;
				// car will need to start braking brakedist before aheadpos
				float brakedist = car.getBrakingDist(ahead.getSpeed()) + World.SPACING;
				float aheaddist = PVector.dist(car.getFrontBumper(),aheadpos);

				//slam the brakes, if need be.
				if ( aheaddist <= brakedist ) {
					steering = PVector.mult(car.getVelocity(),-1);
					steering.setMag(car.getMaxBrake());
					car.setBraking(true);
					return steering;
				}

				while (true) {
					//if path is empty, generate some nodes.
					if (path_.size() == 0) {
						this.generateGraph(car,new RoadGraphNode(car.getRoad(),car.getCenter(),0));
					}
					//if this is true, we have a lane change node we can check out.
					else if (path_.get(path_.size() - 1).getGraphNode().getLane() != car.getLane()) {
						break;
					}	
					else {
						path_.remove(path_.size() - 1);
					}
				}

				/*
				 * now, we want to change to the lane of our next node.
				 */

				target_ = car.getRoad().getLane(path_.get(path_.size() - 1).getGraphNode().getPosition());	
				state_ = State.ChangingLanes;
				lanechange_ = new ChangeLanes(123,target_);
				Signal dir = target_ > car.getLane() ? Signal.RIGHT : Signal.LEFT;		
				car.setSignal(dir);

				//before returning the force, add our position to past_.

				this.addPastNode(new CostNode(new RoadGraphNode(car.getRoad(),car.getCenter(),0),past_.get(past_.size() - 1),0));
				return lanechange_.getSteeringForce(car,world);
			}
			//if we end up here, car ahead is at least twice our neighborhood; just go straight
			else {
				state_ = State.Fwd_MaxSpeed;
				car.setBraking(false);
				track = new TrackLane(world.getApplet().color(255,0,225));
				return track.getSteeringForce(car,world);
			}

			/*
			 * when we're in this state, we're in the middle of changing lanes.
			 * 
			 * we exit this state if:
			 * 		-lane change is done,
			 * 		OR
			 * 		-lane change is aborted (unsafe)
			 */
		case ChangingLanes:
			car.setBraking(false);

			//if not in target lane yet, check to see if still safe.
			if ( car.getLane() != target_ && lanechange_ != null ) {
				for ( Car other : world.getNeighbors(car) ) {

					//if this returns true, must abort lane change.
					//add current to past_, and regenerate graph.
					if ( other.getLane() == target_ ) {

						target_ = -1;
						car.setSignal(Signal.NONE);
						abort = new PVector(lanechange_.getSteeringForce(car,world).x,
						                    -lanechange_.getSteeringForce(car,world).y);
						state_ = State.Braking;
						car.setBraking(true);
						
						//adding our position to past path, and regenerating graph.

						this.addPastNode(new CostNode(new RoadGraphNode(car.getRoad(),car.getCenter(),0),past_.get(past_.size() - 1),0));
						this.generateGraph(car,new RoadGraphNode(car.getRoad(),car.getCenter(),0));

						lanechange_ = null;
						return abort;
					}
				}
			}
			//still safe; continue with our lane change.

			PVector steer = lanechange_.getSteeringForce(car,world);
			if ( car.getLane() != target_ ) { // still changing lanes
				return steer;
			} else { // lane change complete. remove first node from path and put in past_
				CostNode node = null;
				if (path_.size() == 0) {
					this.generateGraph(car,new RoadGraphNode(car.getRoad(),car.getCenter(),0));
					node = path_.remove(0);
					past_.add(0,node);
				}
				else {
					
				}
				
				car.setSignal(Signal.NONE);
				lanechange_ = null;
				target_ = -1;
				state_ = State.Fwd_MaxSpeed;
				track = new TrackLane(world.getApplet().color(255,0,225));
				return track.getSteeringForce(car,world);
			}			

		default:
			System.out.println("default");
			return new PVector(1,0);
		}
	}

	/**
	 * Setter for the goal. Note that this value is set in Traffic's setup()
	 * method.
	 * 
	 * @param goal
	 *          Goal of the smart car.
	 */
	public void setGoal ( Goal goal ) {
		goal_ = goal;
	}

	/**
	 * Used to initialized the DynamicPathfinderGraph.
	 * 
	 * @param world
	 *          Our world
	 * @param car
	 *          The smart car
	 */
	public void initGraph ( World world, Car car ) {
		graph_ = new DynamicPathfinderGraph(car,world,goal_);
	}

	/**
	 * Get the int value of the lane we're changing into
	 * 
	 * @return int value of the lane we're going to
	 */
	public int getTargetLane () {
		return target_;
	}

	/**
	 * Set the target lane of the car.
	 * 
	 * @param target
	 *          new value for target_
	 */
	public void setTargetLane ( int target ) {
		target_ = target;
	}

	/**
	 * Used to set the start time of a simulation.
	 * 
	 * @param start
	 *          start time, long
	 */
	public void setStartTime ( long start ) {
		start_ = start;
	}

	/**
	 * Returns the simulation start time.
	 * 
	 * @return start time of the simulation.
	 */
	public float getStartTime() {
		return start_;
	}

	/**
	 * This method adds the next set of RoadGraphNodes to our PQ.
	 * 
	 * @param car
	 * 		the smart car
	 * 
	 * @param pos
	 * 		position of the node we're generating from.
	 */
	public void addNextNodes(Car car, CostNode parent) {
		float elapsed = System.nanoTime() - getStartTime();

		//note that the cost of the node we feed in is 0, as it is our current position.
		for (RoadGraphNode node: graph_.getNextLocations(parent.getGraphNode())) {

			float etaGoal = ((PVector.sub(goal_.getPoint(),node.getPosition()).mag())
					/ car.getMaxSpeed());			
			float total = elapsed + node.getTime() + etaGoal;
			CostNode insert=new CostNode(node,parent,total);
			if(chooseNode(insert)!=null) {
				pq_.add(chooseNode(insert));
			}

		}
	}

	/**
	 * 
	 * @param node
	 * @return
	 */
	public CostNode containsNode(CostNode node) {

		for (CostNode n : pq_) {
			if (PVector.dist(node.getGraphNode().getPosition(),n.getGraphNode().getPosition()) == 0) {
				return n;
			}
		}
		return null;
	}
	public CostNode chooseNode(CostNode insert) {
		CostNode target=containsNode(insert);
		if(target==null ) {
			return insert;
		}
		if(target.compareTo(insert)==1) {
			pq_.remove(target);
			return insert;
		}
		else {
			return null;
		}
	}

	/**
	 * This method is used to add nodes to the path that we have traveled,
	 * so we can generate our path at the end.
	 * 
	 * @param node
	 * 		node we're adding to our path.
	 */
	public void addPastNode(CostNode node) {
		past_.add(0,node);
		if (past_.size() == 1) {
			past_.get(0).setParent(null);
		}
		//parent of the node is the successor
		else {
			past_.get(0).setParent(past_.get(1));
		}
	}


	/**
	 * This method generates a path to the goal, from the current node position.
	 * @param car
	 * 		the smart car
	 * 
	 * @param node
	 * 		the node we're generating from
	 */
	public void generateGraph(Car car, RoadGraphNode node) {

		float stamp = System.nanoTime();
		float currentTime = -1;

		//clear current path
		path_.clear();//clear the now irrelevant path
		pq_.clear();//just in case, but should be empty anyway
		pq_.add(new CostNode(node,null,node.getTime()));//add the root node
		//while node is not the goal, generate the graph.
		CostNode current = null;
		
		//remove the lowest rank node, and add this node to the current path.
		while (!pq_.isEmpty()) {
			currentTime = System.nanoTime();

			//if our graph takes 3 seconds to generate, break the loop, and try again later.
			if ((currentTime - stamp)/(Math.pow(10,9)) > 3.0f) {
				break;
			}
			current = pq_.remove();

			//if we have reached our goal, add that last node to path and break the loop.
			if (graph_.goalReached(current.getGraphNode())) {
				path_.clear();
				
				//pushing all nodes to the back for convenience
				while (current.getParent() != null) {
					path_.add(0,current);
					current = current.getParent();
				}
				path_.add(0,new CostNode(current.getGraphNode(),null,0));
				break;					
			}

			this.addNextNodes(car,current);

			path_.add(current);
		}
	}

	private void debugPath ( List<CostNode> path, World world ) {

		PApplet applet = world.getApplet();

		applet.ellipseMode(PApplet.CENTER);
		for ( int i = 0 ; i < path.size() - 1 ; i++ ) {
			// System.out.println("drawing path "+path_.get(i));
			PVector p1 = path.get(i).getGraphNode().getPosition(),
					p2 = path.get(i + 1).getGraphNode().getPosition();
			applet.stroke(0,255,255);
			applet.fill(0,255,255);
			applet.line(p1.x,p1.y,p2.x,p2.y);
			applet.ellipse(p2.x,p2.y,10,10);
		}
	}

}
