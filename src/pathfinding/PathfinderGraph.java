package pathfinding;

import java.util.ArrayList;
import java.util.List;

import core.Car;
import core.Goal;
import core.World;
import processing.core.PApplet;
import processing.core.PVector;

/***
 * The pathfinding graph. The graph is generated lazily - nodes are added by
 * calling getNextLocations().
 */
public abstract class PathfinderGraph {

	protected Car car_;
	protected World world_;
	protected Goal goal_;
	
	protected List<RoadGraphNode[]> edges_;  // the generated graph

	/**
	 * Generate the successor nodes (neighbors) for the specified node.
	 * 
	 * @param node
	 *          current node
	 * @return the successor nodes for the specified node
	 */
	public abstract List<RoadGraphNode> getNextLocations ( RoadGraphNode node );

	/**
	 * Create a new (empty) pathfinding graph for the specified car, world, and
	 * goal.
	 * 
	 * @param car
	 * @param world
	 * @param goal
	 */
	public PathfinderGraph ( Car car, World world, Goal goal ) {
		car_ = car;
		world_ = world;
		goal_ = goal;
		edges_ = new ArrayList<RoadGraphNode[]>();
	}

	/**
	 * Get the start node for the graph.
	 * @return start node
	 */
	public RoadGraphNode getStart () {
		return new RoadGraphNode(car_.getRoad(),car_.getRoad()
		    .getOnCenterStripe(car_.getFrontBumper()),0);
	}

	/**
	 * Determine if the specified node is a goal.
	 * @param node
	 * @return true if the node is a goal, false if not
	 */
	public boolean goalReached ( RoadGraphNode node ) {
		return goal_.matches(node.getPosition());
	}

	/**
	 * Display the generated graph.
	 */
	public void debug () {
		if ( !world_.getDebug(World.DEBUG_GRAPH) ) {
			return;
		}
	
		PApplet applet = world_.getApplet();
	
		if ( goal_ != null ) {
			applet.noFill();
			applet.strokeWeight(3);
			applet.stroke(car_.getColor());
			applet.ellipseMode(PApplet.CENTER);
			applet.ellipse(goal_.getPoint().x,goal_.getPoint().y,
			               2 * goal_.getRadius(),2 * goal_.getRadius());
			applet.strokeWeight(1);
		}
	
		applet.ellipseMode(PApplet.CENTER);
		applet.stroke(car_.getColor());
		applet.fill(0);
		for ( RoadGraphNode[] edge : edges_ ) {
			PVector p1 = edge[0].getPosition(), p2 = edge[1].getPosition();
			// System.out.println("drawing "+p1+" "+p2);
			applet.line(p1.x,p1.y,p2.x,p2.y);
		}
		for ( RoadGraphNode[] edge : edges_ ) {
			PVector p2 = edge[1].getPosition();
			// System.out.println("drawing "+p2);
			applet.ellipse(p2.x,p2.y,10,10);
		}
	
	}

	protected void addEdge ( RoadGraphNode[] nodes ) {
		edges_.add(nodes);
	}

}