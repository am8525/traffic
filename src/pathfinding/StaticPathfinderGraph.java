package pathfinding;

import java.util.ArrayList;
import java.util.List;

import core.Car;
import core.Goal;
import core.Road;
import core.World;
import processing.core.PVector;

public class StaticPathfinderGraph extends PathfinderGraph {

	public StaticPathfinderGraph ( Car car, World world, Goal goal ) {
		super(car,world,goal);
	}

	private void addLaneChange ( RoadGraphNode node, int newlane,
	                             List<RoadGraphNode> targets ) {
		Road road = node.getRoad();

		// compute the target location in the new lane
		PVector target = road.getLaneChangeTarget(node.getPosition(),newlane,
		                                          Car.LANE_CHANGE_ANGLE);

		// check that there is room in the new lane for the car - one car length
		// back from target, with spacing on either end
		if ( world_.isIntervalVacant(road,target,-car_.getLength() - World.SPACING,
		                             World.SPACING) ) {
			float dist = PVector.dist(car_.getFrontBumper(),target);
			RoadGraphNode newnode =
			    new RoadGraphNode(road,target,
			                      node.getTime() + dist / car_.getMaxSpeed());
			targets.add(newnode);
			world_.debugPoint(World.DEBUG_GRAPHGEN,target,
			                  world_.getApplet().color(0,255,255),8);
			// System.out.println("lane change node (" + newlane + "): " + newnode);
		}

	}

	private void addOpportunity ( RoadGraphNode node, int newlane, Car ahead,
	                              List<RoadGraphNode> targets ) {
		// in the current lane, immediately after passing each car in the adjacent
		// lane, until reaching the next car in the current lane
		// safe opportunity - car travels at max speed until braking just before
		// next car in lane
		// unsafe opportunity - car travels at max speed

		Road road = node.getRoad();
		int lane = road.getLane(node.getPosition());
		float time = node.getTime();

		PVector target = road.getLaneChangeTarget(node.getPosition(),newlane,
		                                          Car.LANE_CHANGE_ANGLE);

		// end of the interval for opportunity nodes (in the current lane)
		PVector end = (ahead == null ? road.getEnd(lane)
		    : road.getAlong(ahead.getRearBumper(),-World.SPACING));
		// start of the interval for adjacent cars (in the specified lane) - end
		// is corresponding point to end
		PVector adjstart = road.getOnCenterStripe(node.getPosition(),newlane);
		PVector adjend = road.getOnCenterStripe(end,newlane);

		for ( Car adjacent : world_
		    .getCarsInInterval(road,adjstart,0,PVector.dist(adjstart,adjend)) ) {
			// (unsafe) opportunity point is the front of the adjacent car + spacing
			// + length of this car

			// is there room in front of the adjacent car? (before the end of the
			// road)
			// where car will end up after pass
			PVector passing = road.getAlong(adjacent.getFrontBumper(),
			                                World.SPACING + car_.getLength());
			// no room to pass the car, so no reason to create an opportunity node
			if ( road.compareTo(passing,road.getEnd(newlane)) > 0 ) {
				continue;
			}

			// adjp is adjacent's location in our lane
			PVector adjp = road.getOnCenterStripe(adjacent.getFrontBumper(),lane);
			// p is the departure location in our lane
			PVector p =
			    road.getAlong(node.getPosition(),PVector.dist(target,passing));
			// departure point is past next car in this lane
			if ( road.compareTo(p,end) > 0 ) {
	//			System.out.println("["+adjacent.getID()+"] departure: "+p+" "+end);
				continue;
			}

			// unsafe opportunity nodes - travel as fast as possible
			RoadGraphNode unsafe = new RoadGraphNode(road,p,time
			    + PVector.dist(node.getPosition(),p) / car_.getMaxSpeed());
			targets.add(unsafe);
			world_.debugPoint(World.DEBUG_GRAPHGEN,unsafe.getPosition(),
			                  world_.getApplet().color(255,0,0),8);
			{
				PVector target2 = road.getLaneChangeTarget(unsafe.getPosition(),newlane,
				                                           Car.LANE_CHANGE_ANGLE);
				world_.debugPoint(World.DEBUG_GRAPHGEN,target2,
				                  world_.getApplet().color(255,0,0),5);
			}
			// System.out
			// .println("unsafe opportunity node (" + newlane + "): " + unsafe);

			// safe opportunity nodes - take into account braking
			if ( ahead != null ) {
				float brakedist = car_.getBrakingDist(ahead.getSpeed());
				// where we need to start braking in our lane
				PVector brakestart =
				    road.getAlong(ahead.getRearBumper(),-World.SPACING - brakedist);
				if ( road.compareTo(brakestart,p) < 0 ) {
					// p is unsafe - must brake before then

					// extra time needed to get to departure point due to braking
					float addlt = (-car_.getMaxSpeed()
					    + (float) Math.sqrt(car_.getMaxSpeed() * car_.getMaxSpeed()
					        - 2 * car_.getMaxBrake() * PVector.dist(brakestart,p)))
					    / -car_.getMaxBrake();
					RoadGraphNode safe = new RoadGraphNode(road,p,time
					    + PVector.dist(node.getPosition(),brakestart) / car_.getMaxSpeed()
					    + addlt);
					targets.add(safe);
					world_.debugPoint(World.DEBUG_GRAPHGEN,safe.getPosition(),
					                  world_.getApplet().color(0,255,0),8);
					{
						PVector target2 =
						    road.getLaneChangeTarget(safe.getPosition(),newlane,
						                             Car.LANE_CHANGE_ANGLE);
						world_.debugPoint(World.DEBUG_GRAPHGEN,target2,
						                  world_.getApplet().color(0,255,0),5);
					}
					// System.out
					// .println("safe opportunity node (" + newlane + "): " + safe);
				}
			}
		}
	}

	private void addBoundary ( RoadGraphNode node, Car ahead,
	                           List<RoadGraphNode> targets ) {
		Road road = node.getRoad();
		int lane = road.getLane(node.getPosition());
		float time = node.getTime();

/*		if ( ahead == null ) {
			// no car, just drive forward at max speed to the end of the road
			PVector end = road.getEnd(lane);
			float drivetime =
			    PVector.dist(node.getPosition(),end) / car_.getMaxSpeed();
			if ( road.compareTo(node.getPosition(),end) < 0 ) {
				RoadGraphNode boundary = new RoadGraphNode(road,end,time + drivetime);
				targets.add(boundary);
				world_.debugPoint(World.DEBUG_GRAPHGEN,boundary.getPosition(),
				                  world_.getApplet().color(0,0,255),8);
				// System.out.println("boundary node: (a) " + boundary);
			}
		} else { */
			// drive as fast as possible, then slow at last minute to max speed of
			// next car
			float brakedist = car_.getBrakingDist(ahead.getSpeed());
			float braketime = car_.getBrakingTime(ahead.getSpeed());
			float availdist = PVector.dist(node.getPosition(),ahead.getRearBumper())
			    - World.SPACING;
			// System.out.println(brakedist+" "+availdist);
			if ( availdist > 0 && brakedist <= availdist ) {
				RoadGraphNode boundary = new RoadGraphNode(road,road.getAlong(ahead
				    .getRearBumper(),-World.SPACING),time
				        + (availdist - brakedist) / car_.getMaxSpeed() + braketime);
				targets.add(boundary);
				world_.debugPoint(World.DEBUG_GRAPHGEN,boundary.getPosition(),
				                  world_.getApplet().color(0,0,255),8);
				// System.out.println("boundary node: (b) " + boundary);
			}
	//	}
	}

	private void addGoal ( RoadGraphNode node, Car ahead,
	                       List<RoadGraphNode> targets ) {
		// add goal if reachable - in the current lane and nothing in the way

		Road road = node.getRoad();
		int lane = road.getLane(node.getPosition());
		float time = node.getTime();

		if ( lane != road.getLane(goal_.getPoint()) ) {
			return;
		}

		float t =
		    PVector.dist(node.getPosition(),goal_.getPoint()) / car_.getMaxSpeed();

		if ( ahead == null ) {
			RoadGraphNode goal = new RoadGraphNode(road,goal_.getPoint(),time + t);
			targets.add(goal);
			world_.debugPoint(World.DEBUG_GRAPHGEN,goal.getPosition(),
			                  world_.getApplet().color(0,255,0),8);
			System.out.println("goal node: " + goal);
		}

	}

	@Override
	public List<RoadGraphNode> getNextLocations ( RoadGraphNode node ) {
		List<RoadGraphNode> targets = new ArrayList<RoadGraphNode>();
		Road road = node.getRoad();
		int lane = road.getLane(node.getPosition());
		float time = node.getTime();

		// can't plan how other cars will change lanes, so expect next car ahead
		// to always be the one that is currently ahead
		Car ahead = world_.getNextCarInLane(road,node.getPosition());
		// System.out.println("ahead: " + ahead);

		// System.out.println("getNextLocations(): "+node);

		// goal node
		addGoal(node,ahead,targets);

		// boundary node - only add if there's a car ahead
		if ( ahead != null ) {
			addBoundary(node,ahead,targets);
		}

		// lane change nodes
		if ( lane > 0 ) {
			addLaneChange(node,lane - 1,targets);
		}
		if ( lane < road.getNumLanes() - 1 ) {
			addLaneChange(node,lane + 1,targets);
		}

		// opportunity nodes - in current lane, immediately after passing each
		// neighboring car, until next car in current lane
		if ( lane > 0 ) {
			addOpportunity(node,lane - 1,ahead,targets);
		}
		if ( lane < road.getNumLanes() - 1 ) {
			addOpportunity(node,lane + 1,ahead,targets);
		}

		// System.out.println(" ---------------- " + location + " " + car_);
		for ( RoadGraphNode target : targets ) {
			addEdge(new RoadGraphNode[] { node, target });
			// System.out.println("neighbors: " + edges_.get(edges_.size() - 1)[1]);
		}
		return targets;
	}

}