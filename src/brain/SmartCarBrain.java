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

	/*
	 * need:
	 * 	-world
	 * 	-car for this brain
	 * 	-goal
	 */
	private DynamicPathfinderGraph graph_; //to be used with A*.

	private boolean pathOn_;

	/*
	 * nodes in the PQ are prioritized by cost,
	 * where cost is : 
	 * 	how far we've traveled thus far
	 * 
	 *  			+
	 *  
	 * 	how far our heuristic tells us this node is from the goal.
	 *  
	 */
	private PriorityQueue<CostNode> pq_;  //PQ for A*

	public SmartCarBrain () {
		pathOn_ = false;
		goal_ = null;
		pq_ = new PriorityQueue<CostNode>();
	}

	public PVector getNetSteeringForce ( Car car, World world ) {

		// TODO: implement this
		if (!pathOn_) {
			pathOn_ = true;
			graph_.debug();
		}
		/*
		 * 1. getNextLocations from our graph
		 * 
		 * 2. for each node:
		 * 		-compute elapsed time thus far.
		 * 		-heuristically compute time it would take to get to the node.
		 * 		-heuristically compute time it would take to get from that node to the goal.
		 * 		
		 * 		-construct a CostNode, and add it to our priority queue.
		 */
		long elapsed = System.nanoTime() - world.getStartTime();
		for (RoadGraphNode node: graph_.getNextLocations(new RoadGraphNode(car.getRoad(),
		                                                                   car.getCenter(),System.nanoTime()))) {

			long etaNode = (long) ((PVector.sub(car.getCenter(),node.getPosition()).mag()) / car.getMaxSpeed());		
			long etaGoal = (long) ((PVector.sub(goal_.getPoint(),node.getPosition()).mag()) / car.getMaxSpeed());			
			long total = elapsed + etaNode + etaGoal;
			pq_.add(new CostNode(node,total));
		}	
		/*
		 * once we've added all the next potential locations,
		 * we know they're ordered by cost. We take the head 
		 * of the list as our best option, cost-wise.
		 */ 

		if (car.getSignal() != Signal.NONE) { 
			/*in middle of lane change.
			 * 
			 * if we have made it to the target lane,
			 * turn off the signal and follow the car in front.
			 * 
			 * if we have not made it yet,
			 * check neighbors. if a neighbor is in the lane we're trying to change to, abort.
			 */


			if (car.getLane() == car.getTargetLane()) { //if true, want to complete lane change until in middle of lane.

				car.setSignal(Signal.NONE);
				car.setTargetLane(-1);
				System.out.println("lane change complete");
				
			}
			else { //must check neighbors to see if we need to abort the lane change.
				for (Car other : world.getNeighbors(car)) {
					if (other.getLane() == car.getTargetLane()) {
						System.out.println("danger! aborting lane change");
						car.setSignal(Signal.NONE);
						car.setTargetLane(-1);
						//						TrackLane abort = new TrackLane(150);
						//						return abort.getSteeringForce(car,world);
						break;
					}
				}
				if (car.getTargetLane() != -1) {
					System.out.println("continuing lane change");
					ChangeLanes changelanes = new ChangeLanes(100,car.getTargetLane()); //continue the lane change
					return changelanes.getSteeringForce(car,world);
				}			
			}
//			Car ahead = world.getNextCarInLane(car.getRoad(),car.getFrontBumper());
//			Follow follow = new Follow(ahead,100);
//			return follow.getSteeringForce(car,world);
			Behavior track = new TrackLane(world.getApplet().color(255,0,225));
			return track.getSteeringForce(car,world);
		}

		ArrayList<RoadGraphNode> options = new ArrayList<RoadGraphNode>(); //used to hold what we pull from PQ
		if (car.isBraking()) {
			while (true) {
				RoadGraphNode node = pq_.remove().getGraphNode();
				if (car.getRoad().getLane(node.getPosition()) != car.getLane()) { //disregard node if in same lane
					options.add(node);
				}
				if (pq_.isEmpty()) { //we've exhausted our options, so we will just break.
					break;
				}
			}
			/*
			 * for a given node, if it is in the same lane as a neighbor, we consider it unsafe 
			 * and move on.
			 */
			for (RoadGraphNode node : options) { 
				boolean isSafe = true; //if this flag remains true after we check all neighbors, we use the node.
				int target = car.getRoad().getLane(node.getPosition());
				for (Car other : world.getNeighbors(car)) {
					if (!isSafe) {
						break;
					}
					if (target == other.getLane()) {
						isSafe = false;
					}
				}
				if (isSafe) { //make the lane change
					ChangeLanes changelanes = new ChangeLanes(50,target);
					if (target < car.getLane()) {
						car.setSignal(Signal.LEFT);
					}
					else {
						car.setSignal(Signal.RIGHT);						
					}		
					System.out.println("making lane change");
					car.setBraking(false);
					car.setTargetLane(target);
					return changelanes.getSteeringForce(car,world);
				}
			}
		}

		for (Car other : world.getNeighbors(car)) {		
			if (other.getLane() == car.getLane()) { //same lane, should break.
				System.out.println("neighbor inlane, breaking");
				car.setBraking(true);
				Brake brake = new Brake(100);
				return brake.getSteeringForce(car,world);
			}
			else {
				System.out.println("neighbor diff lane, not breaking");
			}
		}
		//if we get here, go straight @ max speed.
		//		Car ahead = world.getNextCarInLane(car.getRoad(),car.getFrontBumper());
		//		if (ahead != null) {
		//			System.out.println("str8");
		//			Follow follow = new Follow(ahead,100);
		//			return follow.getSteeringForce(car,world);
		//		}
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
