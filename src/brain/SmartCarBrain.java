package brain;

import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;

import core.Brain;
import core.Car;
import core.Goal;
import core.World;
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
	private PriorityQueue<RoadGraphNode> pq_; //PQ for A*

	public SmartCarBrain () {
		goal_ = null;

	}

	public PVector getNetSteeringForce ( Car car, World world ) {
		
		// TODO: implement this
		
		/*
		 * 1. getNextLocations from our graph
		 */
		for (RoadGraphNode node: graph_.getNextLocations(new RoadGraphNode(car.getRoad(),
		                                                         car.getCenter(),System.nanoTime()))) {
			
		}
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
