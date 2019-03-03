package brain;

import java.util.List;
import java.util.PriorityQueue;

import core.Brain;
import core.Car;
import core.Goal;
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
	private PriorityQueue<CostNode> pq_; //PQ for A*

	public SmartCarBrain () {
		goal_ = null;
		pq_ = new PriorityQueue<CostNode>();
	}

	public PVector getNetSteeringForce ( Car car, World world ) {
		
		// TODO: implement this
		
		/*
		 * 1. getNextLocations from our graph
		 * 
		 * 2. for each node:
		 * 		-compute elapsed time thus far.
		 * 		-heuristically compute time it would take to get to the node.
		 * 		-heuristically compute time it would take to get from that node to the goal.
		 * 		
		 * 		-construct a CostNode, and add it to our priority queue.
		 * 
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
		
		
		 /* TODO: implement a decision-making procedure 
		 * to pick a node, and return a force directed toward it.
		 */
		return new PVector(0,0);
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
