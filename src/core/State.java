package core;

/**
 * @author ajm
 * These are the possible states 
 * our smart car may be in.
 */
public enum State {
	
	Fwd_MaxSpeed, //in this state, we are going straight away in the current lane, max speed.
	
	Braking, //a car is ahead in our AOE; looking to make a lane change, if possible.
	
	ChangingLanes, //in the midst of a lane change; continue changing if safe still, or go back to current if unsafe.
	
	CloseToGoal //enter this state when within a certain threshold of the end of the screen; must get into middle lane.
	
}
