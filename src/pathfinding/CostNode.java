package pathfinding;

import processing.core.PVector;

/**
 * @author ajm
 *
 */
public class CostNode implements Comparable<CostNode> {

	//associated RoadGraphNode
	private RoadGraphNode node_; 
	
	private CostNode parent_;
	
	//cost it takes to get to this node.
	private float cost_;
	
	public CostNode(RoadGraphNode node, CostNode parent, float cost) {
		node_ = node;
		cost_ = cost;
		parent_ = parent;
	}
	public CostNode(RoadGraphNode node,float cost) {
		node_ = node;
		cost_ = cost;
	}
	
	/**
	 * Necessary function for CostNode to work with PriorityQueue.
	 * 
	 * CostNodes must be ordered by their cost in ascending order,
	 * as lower costs are favored.
	 * 
	 * @param c
	 * 		CostNode we're comparing this CostNode with.
	 * 
	 * @return
	 * 		-1 if the cost_ < c.getCost()
	 * 		0 if cost_ == c.getCost()
	 * 		1 if cost_ > c.getCost()
	 */
	public int compareTo ( CostNode c ) {
		if (cost_ < c.getCost()) {
			return -1;
		}
		else if (cost_ > c.getCost()) {
			return 1;
		}
		else {
			return 0;
		}	
	}
	
	/**
	 * Setter for cost_
	 * @param cost
	 * 			new cost to get to this node.
	 */
	public void setCost(float cost) {
		cost_ = cost;
	}
	
	/**
	 * Getter for cost_
	 * @return
	 * 		cost to get to this node.
	 */
	public float getCost() {
		return cost_;
	}
	
	/**
	 * Setter for the parent of this CostNode
	 * 
	 * @param node
	 * 		new parent of this node
	 */
	public void setParent(CostNode node) {
		parent_ = node;
	}
	
	/**
	 * Getter for parent of this CostNode.
	 * 
	 * @return the parent node
	 */
	public CostNode getParent() {
		return parent_;
	}
	
	public RoadGraphNode getGraphNode() {
		return node_;
	}

}
