package ca.mcgill.ecse211.lab4;
import lejos.hardware.sensor.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;
import lejos.hardware.Button;

/**
 * @author Ahmed Elehwany 260707540, Barry Chen
 * class for navigation between points without using obstacle avoidance.
 *
 */
public class Navigation implements Runnable {

	public static EV3LargeRegulatedMotor leftMotor;
	public static EV3LargeRegulatedMotor rightMotor;
	private final double TRACK;
	private final double WHEEL_RAD;
	private Odometer odometer;
	private OdometerData odoData;
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	private static final double tileLength = 30.48;
	private static double prevAngle = 0;
	double currentT, currentY, currentX;
	double dx, dy;
	double t;
	double distance;

	/**
	 * creates a navigation instance.
	 * @param leftMotor left motor object
	 * @param rightMotor right motor object
	 * @param TRACK track value
	 * @param WHEEL_RAD wheel radius
	 * @param finalPath map coordinates list
	 * 
	 * @throws Exception
	 */
	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
		      final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
		this.odometer = Odometer.getOdometer();
	    this.leftMotor = leftMotor;
	    this.rightMotor = rightMotor;
	    odoData = OdometerData.getOdometerData();
	    odoData.setXYT(0 , 0 , 0);
	    this.TRACK = TRACK;
	    this.WHEEL_RAD = WHEEL_RAD;
	}

	/**
	 * method to start moving the car in navigation mode
	 * @return void
	 */
	public void run() {
		// wait 5 seconds
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
			motor.stop();
			motor.setAcceleration(300);
		}
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// there is nothing to be done here because it is not expected that
			// the odometer will be interrupted by another thread
		}
		// implemented this for loop so that navigation will work for any number of points
		
	}
	
	/**
	 * causes  the  robot  to  travel  to  the  absolute  field  location  (x,  y),  
	 * specified  in tilepoints.This  method  should  continuously  callturnTo(double theta)
	 * and  then set  the motor speed to forward(straight). This will make sure that your heading is updated
	 * until you reach your exact goal. This method will poll the odometer for information.
	 * @param x x-coordinate
	 * @param y y-coordinate 
	 * @return void
	 */
	void travelTo(double x, double y) {
		//get the current position of the robot
		currentX = odometer.getXYT()[0];
		currentY = odometer.getXYT()[1];
		currentT = odometer.getXYT()[2];
		
		dx = x - currentX;
		dy = y - currentY;
		distance = Math.sqrt(dx*dx + dy*dy);
		if(dy>=0) {
			t=Math.atan(dx/dy);
		}
		else if(dy<=0 && dx>=0) {
			t = Math.atan(dx/dy) + Math.PI;
		}
		else {
			t = Math.atan(dx/dy) - Math.PI;
		}//Mathematical convention
		
		// initial angle is 0||2pi, same direction as y-axis, going clockwise
		double differenceInTheta = (t*180/Math.PI-currentT); // robot has to turn "differenceInTheta",
		//turn the robot to the desired direction
		turnTo(differenceInTheta); 
		
		// drive forward required distance
	    leftMotor.setSpeed(FORWARD_SPEED);
	    rightMotor.setSpeed(FORWARD_SPEED);
	    leftMotor.rotate(convertDistance(WHEEL_RAD, distance), true);
	    rightMotor.rotate(convertDistance(WHEEL_RAD, distance), false);
	}
	
	/**
     * This method uses the current angle as a reference
     * to turn to the angle required
     * @param theta: angle to turn by
     * @throws OdometerExceptions
     */
    public static void turnWithTheta(double theta) throws OdometerExceptions {

    	boolean turnLeft = false;
		double deltaAngle = 0;
		// Get change in angle we want
		prevAngle = Odometer.getOdometer().getXYT()[2];
		deltaAngle = theta - prevAngle;

		// If deltaAngle is negative, loop it back
		if (deltaAngle < 0) {
			deltaAngle = 360 - Math.abs(deltaAngle);
		}

		// Check if we want to move left or right
		if (deltaAngle > 180) {
			turnLeft = true;
			deltaAngle = 360 - Math.abs(deltaAngle);
		} else {
			turnLeft = false;
		}

		// Set slower rotate speed
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// Turn motors according to which direction we want to turn in
		if (turnLeft) {
			leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, deltaAngle), true);
			rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, deltaAngle), false);
		} else {
			leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, deltaAngle), true);
			rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, deltaAngle), false);
		}

    }
	
	
	/**
	 *	This method causes the robot to turn (on point) to the absolute heading theta. 
	 *  This method should turn a MINIMAL angle to its target.
	 *  @param theta turn angle value
	 *  @return void
	 */
	public static void turnTo(double theta) {
		if(theta>180) {//angel convention, turn in correct minimal angle
			theta=360-theta;
			leftMotor.setSpeed(ROTATE_SPEED);
		    rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, theta), true);
			rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, theta), false);
			}
		else if(theta<-180) {
			theta=360+theta;
			leftMotor.setSpeed(ROTATE_SPEED);
		    rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, theta), true);
			rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, theta), false);
			}
		else {
			leftMotor.setSpeed(ROTATE_SPEED);
		    rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, theta), true);
			rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, theta), false);	
		}
	}
	
	/**
	 * This  method  returns  true  if  another  thread  has  called travelTo()
	 * or turnTo()and  the method has yet to return; false otherwise.    
	 * @return boolean 
	 */
	boolean isNavigating() {
	 if((leftMotor.isMoving() && rightMotor.isMoving()))
		 return true;
	 else 
		 return false;
	}
	
	/**
	 * This  method  converts target distance to wheel rotation.
	 * @param radius radius of wheel
	 * @param distance target distance
	 * @return the wheel rotation 
	 */
	 public static int convertDistance(double radius, double distance) {
		    return (int) ((180.0 * distance) / (Math.PI * radius));
		  }
 	/**
	 * This  method  converts target distance to wheel rotation.
	 * @param radius radius of wheel
	 * @param width track value
	 * @param angle target turn angle 
	 * @return the wheel rotation 
	 */
	 public static int convertAngle(double radius, double width, double angle) {
		    return convertDistance(radius, Math.PI * width * angle / 360.0);
	}


}
