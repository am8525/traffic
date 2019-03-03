import java.util.Random;

import brain.PassingCarBrain;
import brain.SimpleCarBrain;
import brain.SmartCarBrain;
import core.Car;
import core.Goal;
import core.Road;
import core.World;
import processing.core.PApplet;
import processing.core.PVector;

/**
 * Traffic simulator main program.
 * 
 * @author ssb
 */
public class Traffic extends PApplet {

	public static void main ( String[] args ) {
		PApplet.main("Traffic");
	}

	private World world_; // the world containing the cars and roads

	private boolean paused_, step_; // animation/simulation controls

	public void draw () {
		if ( !paused_ || step_ ) {
			if (world_.getStartTime() == -1) { //setting the start time
				System.out.println("time set");
				world_.setStartTime(System.nanoTime());
			}
			background(50); // clear background

			// draw the world
			world_.render();

			// calculate steering forces and move all cars
			world_.update();

			step_ = false;
		}
	}

	public void keyPressed () {
		if ( key == 'p' ) {
			paused_ = !paused_;
			if (paused_) {
				
			}
		} else if ( key == 's' ) {
			paused_ = true;
			step_ = true;
			
		} else if ( key == 'b' ) {
			world_.setDebug(World.DEBUG_BEHAVIOR,!world_.getDebug(World.DEBUG_BEHAVIOR));
		} else if ( key == 'G' ) {
			world_.setDebug(World.DEBUG_GRAPHGEN,!world_.getDebug(World.DEBUG_GRAPHGEN));
		} else if ( key == 'g' ) {
			world_.setDebug(World.DEBUG_GRAPH,!world_.getDebug(World.DEBUG_GRAPH));
		} else if ( key == 'c' ) {
			world_.setDebug(World.DEBUG_CAR,!world_.getDebug(World.DEBUG_CAR));
		}
	}

	/**
	 * Find a random location for a car which doesn't overlap with any other cars
	 * in the world.
	 * 
	 * @param road
	 *          road the car is on
	 * @param carlength
	 *          length of the car
	 * @return "safe" position for the car
	 */
	private PVector placeCar ( Road road, int carlength ) {
		Random random = new Random();
		for ( ; true ; ) {
			int lane = random.nextInt(road.getNumLanes());
	//		int lane = 2;
			PVector p =
			    road.getAlong(road.getStart(lane),
			                  random.nextFloat()
			                      * (road.getLength() - carlength - 2 * World.SPACING)
			                      + World.SPACING);
			// System.out.println("\t"+p+" "+road.getStart(lane));

			// position is front bumper
			if ( world_.isIntervalVacant(road,p,-carlength - World.SPACING,
			                             World.SPACING) ) {
				return p;
			}
		}
	}

	public void settings () {
		size(1400,200);
	}

	public void setup () {
		paused_ = true; // start with the simulation paused
		step_ = false;

		world_ = new World(this);

		Random random = new Random();

		// one road
		Road road = new Road(new PVector(0,25),new PVector(1400,25),5,30);
		world_.addRoad(road);

		// one smart car
		SmartCarBrain brain = null;
		Car smartCar = null;
		{
			brain = new SmartCarBrain();
			brain.setGoal(new Goal(road.getEnd(2),15));
			smartCar =
			    new Car(world_,color(255,255,0),10,20,1,road.getAlong(road.getStart(2),10),
			            new PVector(0,0),4f,.05f,1f,40,PApplet.radians(135),brain);
			smartCar.setRoad(road);
			world_.addCar(smartCar);
		}

		// 20 randomly-placed other cars
		for ( int ctr = 0 ; ctr < 10 ; ctr++ ) {
			float speed = random.nextFloat() + .5f;
			Car car = new Car(world_,color(255,0,255),10,20,1,placeCar(road,20),
			                  new PVector(0,0),1f,.05f,speed,40,PApplet.radians(135),
			                  new SimpleCarBrain());
			car.setRoad(road);
			world_.addCar(car);
		}
		for ( int ctr = 0 ; ctr < 10 ; ctr++ ) {
			float speed = random.nextFloat() + .5f;
			Car car = new Car(world_,color(0,0,255),10,20,1,placeCar(road,20),
			                  new PVector(0,0),1f,.05f,speed,40,PApplet.radians(135),
			                  new PassingCarBrain());
			car.setRoad(road);
			world_.addCar(car);
		}
		
		/*
		 * initialize the DynamicPathfinderGraph inside of SmartCarBrain
		 */
		brain.initGraph(world_,smartCar);

		// so something is visible when it starts off paused
		background(50);
		world_.render();
	}
}
