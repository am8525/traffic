package brain;

import java.util.List;

import core.Brain;
import core.Car;
import core.Goal;
import core.World;
import pathfinding.RoadGraphNode;
import processing.core.PApplet;
import processing.core.PVector;

/**
 * A smart car, which has a goal and can navigate its way around other traffic.
 */
public class SmartCarBrain implements Brain {

	private Goal goal_;

	public SmartCarBrain () {
		goal_ = null;
	}

	public PVector getNetSteeringForce ( Car car, World world ) {
		// TODO: implement this
		return new PVector(0,0);
	}

	public void setGoal ( Goal goal ) {
		goal_ = goal;
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
