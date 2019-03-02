package pathfinding;

import core.Road;
import processing.core.PVector;

public class RoadGraphNode {
	private Road road_;
	private PVector p_;
	private float time_;

	public RoadGraphNode ( Road road, PVector p, float time ) {
		super();
		road_ = road;
		p_ = p;
		time_ = time;
	}

	public int getLane () {
		return road_.getLane(p_);
	}

	public PVector getPosition () {
		return p_;
	}

	public Road getRoad () {
		return road_;
	}

	public float getTime () {
		return time_;
	}

	@Override
	public String toString () {
		return "RoadGraphNode [p_=" + p_ + ", time_=" + time_ + "]";
	}

}
