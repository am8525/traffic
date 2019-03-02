package core;

import processing.core.PApplet;
import processing.core.PVector;

/**
 * A straight road. (It isn't necessarily horizontal or vertical, but there
 * aren't curves or corners.) Roads are always one-way, and are drawn so that
 * the left side of the road is along the line between start and end. (This is
 * done so that it is easy to put two one-way roads next to each other to make a
 * two-way road.)
 * 
 * @author ssb
 */
public class Road {

	/**
	 * Describes a position along a road in terms of its lane number and the
	 * distance along the road (from the start).
	 */
	class RoadPosition {

		private float t_; // distance along the road (0 = at start, 1 = at end)
		private int lane_; // lane number (0..numlanes-1)

		/**
		 * Create a road position.
		 * 
		 * @param lane
		 *          the lane number (0..numlanes-1)
		 * @param t
		 *          distance along the road (0 = at start, 1 = at end)
		 */
		RoadPosition ( int lane, float t ) {
			super();
			lane_ = lane;
			t_ = t;
		}

		/**
		 * Compute the local road coordinates equivalent of p.
		 * 
		 * @param p
		 *          point
		 * @return p, in road coordinates
		 */
		RoadPosition ( PVector p ) {
			PVector a = start_; // point on the line
			PVector n = getOrientation(); // unit vector pointing along the road

			if ( ccw(start_,p,end_) > 0 ) {
				// outside the road
				lane_ = -1;
				return;
			}

			// (a-p)-((a-p).n)n
			PVector amp = PVector.sub(a,p);
			PVector perp = PVector.sub(amp,PVector.mult(n,PVector.dot(amp,n)));
			PVector closest = PVector.add(p,perp);

			// calculate y and dist along the road
			float y = perp.mag();
			t_ = PVector.sub(closest,start_).mag() / getLength();

			if ( y > lanewidth_ * numlanes_ ) {
				// outside the road
				lane_ = -1;
				return;
			}

			lane_ = (int) (y / lanewidth_);
		}

		/**
		 * Get the coordinates (on the center stripe) for this position.
		 * 
		 * @return coordinates (on the center stripe) for this position
		 */
		PVector getAsCoords () {
			if ( lane_ == -1 ) {
				return null;
			}

			// center of lane, from edge of road along perpendicular
			float y = (lane_ + .5f) * lanewidth_;

			PVector edge = PVector.sub(end_,start_);
			PVector perp =
			    PVector.fromAngle(edge.heading() + PApplet.radians(90)).normalize();

			PVector edgept = PVector.add(start_,PVector.mult(edge,t_));
			// System.out.println("edge "+edge+" / perp "+perp+" / edgept "+edgept);
			return PVector.add(edgept,PVector.mult(perp,y));
		}

		/**
		 * Get the lane number.
		 * 
		 * @return the lane number (0..numlanes-1)
		 */
		int getLane () {
			return lane_;
		}

		/**
		 * Get the distance along the road.
		 * 
		 * @return the distance along the road (0 = at start, 1 = at end)
		 */
		float getT () {
			return t_;
		}

		@Override
		public String toString () {
			return "[RoadPosition " + lane_ + " " + t_ + "]";
		}
	}

	// width of a lane, in pixels
	protected float lanewidth_;

	// number of lanes
	protected int numlanes_;

	// endpoints of road (along the left side of the road)
	private PVector start_, end_;

	/**
	 * Create a new road.
	 * 
	 * @param start
	 *          start of the left side of the road
	 * @param end
	 *          end of the left side of the road
	 * @param numlanes
	 *          number of lanes
	 * @param lanewidth
	 *          width of one lane, in pixels
	 */
	public Road ( PVector start, PVector end, int numlanes, float lanewidth ) {
		start_ = start;
		end_ = end;
		numlanes_ = numlanes;
		lanewidth_ = lanewidth;
	}

	private float ccw ( PVector a, PVector b, PVector c ) {
		return (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
	}

	/**
	 * Return -1 if a is before b along the road (i.e. closer to the start), 1 if
	 * a is after b, and 0 otherwise.
	 * 
	 * @param a
	 * @param b
	 * @return -1 if a is before b along the road (i.e. closer to the start), 1 if
	 *         a is after b, and 0 otherwise
	 */
	public int compareTo ( PVector a, PVector b ) {
		RoadPosition ar = new RoadPosition(a), br = new RoadPosition(b);
		if ( ar.getT() < br.getT() ) {
			return -1;
		} else if ( ar.getT() > br.getT() ) {
			return 1;
		} else {
			return 0;
		}
	}

	/**
	 * Get the point 'offset' distance along the road from p.
	 * 
	 * @param p
	 *          point
	 * @param offset
	 *          distance to travel from p (<0 means to get a point before p on the
	 *          road)
	 * @return the point 'offset' distance along the road from p
	 */

	public PVector getAlong ( PVector p, float offset ) {
		return PVector.add(p,PVector.mult(getOrientation(),offset));
	}

	/**
	 * Get the point at the end of the specified lane (on the center stripe).
	 * 
	 * @param lane
	 *          the lane
	 * @return the point at the end of the specified lane (on the center stripe)
	 */
	public PVector getEnd ( int lane ) {
		return (new RoadPosition(lane,1)).getAsCoords();
	}

	/**
	 * Determine which lane p is in.
	 * 
	 * @param p
	 *          a point in the world
	 * @return which lane p is in, or -1 if p is not on the road
	 */
	public int getLane ( PVector p ) {
		return (new RoadPosition(p)).getLane();
	}

	/**
	 * Determine target point in the new lane.
	 * 
	 * @param p
	 *          current position
	 * @param newlane
	 *          destination lane
	 * @param angle
	 *          lane change angle
	 */
	public PVector getLaneChangeTarget ( PVector p, int newlane, float angle ) {
		int lane = getLane(p);

		// lane change heading
		PVector v = getOrientation().normalize(null);
		if ( newlane > lane ) {
			v.rotate(angle);
		} else {
			v.rotate(-angle);
		}

		// intersect v with center stripe in target lane
		PVector p0 = (new RoadPosition(newlane,0)).getAsCoords(); // center stripe
		PVector pv = getOrientation().normalize(null);
		PVector q0 = p; // car
		PVector qv = v;

		float t = (qv.x * (q0.y - p0.y) + qv.y * (p0.x - q0.x))
		    / (pv.y * qv.x - pv.x * qv.y);

		// System.out.println(qv + " " + " / " + t);

		return PVector.add(p0,PVector.mult(pv,t));
	}

	/**
	 * Get the width of a lane.
	 * 
	 * @return width of a lane, in pixels
	 */
	public float getLaneWidth () {
		return lanewidth_;
	}

	/**
	 * Get the length of the road.
	 * 
	 * @return length of the road
	 */
	public float getLength () {
		return PVector.dist(start_,end_);
	}

	/**
	 * Get the number of lanes.
	 * 
	 * @return number of lanes
	 */
	public int getNumLanes () {
		return numlanes_;
	}

	/**
	 * Get the closest point to p along the center stripe of the lane p is in.
	 * 
	 * @param p
	 *          point
	 * @return closest point to p along the center stripe of the lane p is in, or
	 *         null if p is not on the road
	 */
	public PVector getOnCenterStripe ( PVector p ) {
		return (new RoadPosition(p)).getAsCoords();
	}

	/**
	 * Get the point 'offset' units from the closest point to p along the center
	 * stripe of the specified lane.
	 * 
	 * @param p
	 *          point
	 * @param lane
	 *          the lane (0..numlanes-1)
	 * @param offset
	 *          offset from p (<0 means the point is before p)
	 * @return point 'offset' units from the closest point to p along the center
	 *         stripe of the specified lane
	 */
	public PVector getOnCenterStripe ( PVector p, int lane ) {
		if ( lane < 0 || lane >= numlanes_ ) {
			throw new IllegalArgumentException("illegal lane; got " + lane);
		}

		RoadPosition roadpos = new RoadPosition(p);
		if ( roadpos.getLane() == -1 ) {
			return null;
		}
		return (new RoadPosition(lane,roadpos.getT())).getAsCoords();
	}

	/**
	 * Get the orientation of the road.
	 * 
	 * @return unit vector pointing along the road from start to end
	 */
	public PVector getOrientation () {
		return PVector.sub(end_,start_).normalize(null);
	}

	/**
	 * Get the point at the beginning of the specified lane (on the center
	 * stripe).
	 * 
	 * @param lane
	 *          the lane
	 * @return the point at the beginning of the specified lane (on the center
	 *         stripe)
	 */
	public PVector getStart ( int lane ) {
		return (new RoadPosition(lane,0)).getAsCoords();
	}

	/**
	 * Draw the road.
	 */
	public void render ( PApplet parent ) {
		parent.rectMode(PApplet.CORNER);
		float angle = getOrientation().heading();
		parent.pushMatrix();
		parent.translate((start_.x + end_.x) / 2,(start_.y + end_.y) / 2);
		parent.rotate(angle);
		parent.translate(0,numlanes_ * lanewidth_ / 2);
		parent.fill(0);
		parent.stroke(0);
		parent.rect((-getLength() / 2),(-numlanes_ * lanewidth_ / 2),getLength(),
		            (numlanes_ * lanewidth_));
		parent.fill(255,255,0);
		parent.rect((-getLength() / 2),(-numlanes_ * lanewidth_ / 2),getLength(),2);
		parent.stroke(255);
		for ( int ctr = 1 ; ctr <= numlanes_ ; ctr++ ) {
			parent.line((-getLength() / 2),
			            (ctr * lanewidth_ - numlanes_ * lanewidth_ / 2),
			            (getLength() / 2),
			            (ctr * lanewidth_ - numlanes_ * lanewidth_ / 2));
		}
		parent.popMatrix();
	}

	public String toString () {
		return "[Road start=" + start_ + ", end=" + end_ + ", numlanes=" + numlanes_
		    + ", lanewidth=" + lanewidth_ + "]";
	}
 
}