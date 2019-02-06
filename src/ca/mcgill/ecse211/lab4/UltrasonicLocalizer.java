package ca.mcgill.ecse211.lab4;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class UltrasonicLocalizer implements UltrasonicController {
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private Odometer odo;
	private Navigation nav;
	static final int MOTOR_SPEED = 100;
	private static final int D_THRESHHOLD = 30;
	private static final int NOISE_MARGIN = 5;
	private static final int FILTER_OUT = 10;
	private static final int ODO_CORRECTION = 0;
	private static double ALPHA = 0;
	private static double BETA = 0;
	private static double ANGLE_CORRECTION = 0;
	private static double FINAL_ANGLE = 0;
	private int filterControl;
	private int distance;
	SensorModes usSensor = new EV3UltrasonicSensor(usPort);                // usSensor is the instance
	private SampleProvider usDistance = usSensor.getMode("Distance");   
	private float[] usData = new float[usDistance.sampleSize()];  
	UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, this);  // Instantiate poller
	
	public UltrasonicLocalizer( Odometer odometer, Navigation nav) throws OdometerExceptions {
		this.odo = Odometer.getOdometer(Lab4.leftMotor, Lab4.rightMotor, Lab4.TRACK, Lab4.WHEEL_RAD);
		Lab4.leftMotor.setSpeed(MOTOR_SPEED);
		Lab4.rightMotor.setSpeed(MOTOR_SPEED);
		this.nav = nav;
//		SensorModes usSensor = new EV3UltrasonicSensor(usPort);                // usSensor is the instance
//		usDistance = usSensor.getMode("Distance");                    // usDistance provides samples 
//		usData = new float[usDistance.sampleSize()];  
//		usDistance.fetchSample(usData,0);
//	    this.distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
		usPoller.start();
	}

	/**
	 * Performs falling edge localization
	 * @throws OdometerExceptions 
	 */
	void fallingEdge() throws OdometerExceptions{

		//Instantiate odometer storage and set theta of odometer to 0
		double[] odometer = {0,0,0};
		boolean isAboveThresh = false;
		Odometer.getOdometer().setTheta(0);

		// Checks orientation or sets orientation to perform localization
		if (readUSDistance() > (D_THRESHHOLD + NOISE_MARGIN)) {
			isAboveThresh = true;
		} else {
			findWallAbove();
			isAboveThresh = true;
		}

		// Find first falling edge
		while (true) {

			// Move forward and get odometer data
			odometer = Odometer.getOdometer().getXYT();
			Navigation.leftMotor.forward();
			Navigation.rightMotor.backward();

			// If is falling and you are above the threshold
			// then store theta as alpha and stop turning
			if (isFalling() && isAboveThresh) {
				Navigation.leftMotor.stop(true);
				Navigation.rightMotor.stop(false);
				ALPHA = odometer[2];
				isAboveThresh = false;
				break;
			}
		}

		// Find second falling edge
		while (true) {

			// Go backwards and get odometer data
			odometer = Odometer.getOdometer().getXYT();
			Navigation.leftMotor.backward();
			Navigation.rightMotor.forward();

			// Set above thresh to true if you are above the threshold 
			if (readUSDistance() > (D_THRESHHOLD + NOISE_MARGIN)) {
				isAboveThresh = true;
			}

			// If is falling and you are above the threshold
			// then store 180-theta as beta and stop turning
			if (isFalling() && isAboveThresh) {
				Navigation.leftMotor.stop(true);
				Navigation.rightMotor.stop(false);
				BETA = odometer[2];
				break;
			}
		}

		//TODO SORT THIS STUFF OUT
		// Alpha and Beta algorithms
		if (ALPHA < BETA) {
			ANGLE_CORRECTION = 45 - ((ALPHA + BETA) / 2); 
		} else {
			ANGLE_CORRECTION = 225 - ((ALPHA + BETA) / 2);
		} 

		// Set theta to 0 to apply correction
		// from current reference angle
		//Odometer.getOdometer().setTheta(0);
		FINAL_ANGLE = 180-(ANGLE_CORRECTION+odometer[2]+ODO_CORRECTION);
		Navigation.turnTo(FINAL_ANGLE);
	}

	/**
	 * Performs rising edge localization
	 * @throws OdometerExceptions 
	 */
	void risingEdge() throws OdometerExceptions{
		//Instantiate odometer storage and set theta of odometer to 0
		double[] odometer = {0,0,0};
		boolean isBelowThresh = false;
		Odometer.getOdometer().setTheta(0);

		// Was getting a weird error when checking orientation here
		// decided to just call findWallBelow() regardless

		// Checks orientation or sets orientation to perform localization
		//		if (readUSDistance() < (D_THRESHHOLD - NOISE_MARGIN)) {
		//			isBelowThresh = true;
		//		} else {
		findWallBelow();
		isBelowThresh = true;
		//		}

		// Find first rising edge
		while (true) {

			// Move forward and get odometer data
			odometer = Odometer.getOdometer().getXYT();
			Navigation.leftMotor.forward();
			Navigation.rightMotor.backward();

			// If is rising and you are below the threshold
			// then store theta as alpha and stop turning
			if (isRising() && isBelowThresh) {
				Navigation.leftMotor.stop(true);
				Navigation.rightMotor.stop(false);
				ALPHA = odometer[2];
				isBelowThresh = false;
				break;
			}
		}

		// Find second rising edge
		while (true) {

			// Move backwards and get odometer data
			odometer = Odometer.getOdometer().getXYT();
			Navigation.leftMotor.backward();
			Navigation.rightMotor.forward();

			// Set below thresh to true if you are below the threshold 
			if (readUSDistance() < (D_THRESHHOLD - NOISE_MARGIN)) {
				isBelowThresh = true;
			}

			// If is rising and you are below the threshold
			// then store 180-theta as beta and stop turning
			if (isRising() && isBelowThresh) {
				Navigation.leftMotor.stop(true);
				Navigation.rightMotor.stop(false);
				BETA = odometer[2];
				break;
			}
		}

		//TODO SORT THIS STUFF OUT
		// Alpha and Beta algorithms
		if (ALPHA > BETA) {
			ANGLE_CORRECTION = 45 - ((ALPHA + BETA) / 2); 
		} else {
			ANGLE_CORRECTION = 225 - ((ALPHA + BETA) / 2);
		} 

		// Set theta to 0 to apply correction
		// from current reference angle
		//Odometer.getOdometer().setTheta(0);
		FINAL_ANGLE = 180-(ANGLE_CORRECTION+odometer[2]+ODO_CORRECTION);
		Navigation.turnTo(FINAL_ANGLE);
	}
	

	/**
	 * Sets orientation of robot so it can perform the localization with falling edges 
	 * Makes sure that you are above detectable threshold (i.e facing far from wall)
	 * before you read for falling edge
	 */
	void findWallAbove() {
		while (true) {
			Navigation.leftMotor.forward();
			Navigation.rightMotor.backward();

			if (readUSDistance() > (D_THRESHHOLD + NOISE_MARGIN)) {
				Navigation.leftMotor.stop(true);
				Navigation.rightMotor.stop(false);
				break;
			}
		}
	}

	/**
	 * Sets orientation of robot so it can perform the localization with rising edges 
	 * Makes sure that you are below detectable threshold (i.e facing near the wall)
	 * before you read for rising edge
	 */
	void findWallBelow() {
		while (true) {
			Navigation.leftMotor.forward();
			Navigation.rightMotor.backward();;

			if (readUSDistance() < (D_THRESHHOLD - NOISE_MARGIN)) {
				Navigation.leftMotor.stop(true);
				Navigation.rightMotor.stop(false);
				break;
			}
		}
	}

	/**
	 * Checks if fallingEdge, i.e distance
	 * drops below the threshold
	 * @return boolean: if fallingEdge or not
	 */
	boolean isFalling() {
		if (readUSDistance() < (D_THRESHHOLD - NOISE_MARGIN)) {
			return true;
		} else {
			return false;
		}
	}

	/**
	 * Checks if risingEdge, i.e distance
	 * goes above the threshold
	 * @return boolean: if risingEdge or not
	 */
	boolean isRising() {
		if (readUSDistance() > (D_THRESHHOLD + NOISE_MARGIN)) {
			return true;
		} else {
			return false;
		}
	}
	
	@Override
	public void processUSData(int distance) {

		// rudimentary filter - toss out invalid samples corresponding to null
		// signal.
		// (n.b. this was not included in the Bang-bang controller, but easily
		// could have).
		//
		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (distance >= 255) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distance = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;
		}

		// Print values

		if(Lab4.isUSLocalizing) {
//			Lab4.lcd.clear();
//			Lab4.lcd.drawString("Distance: " + distance, 0, 1);
//			Lab4.lcd.drawString("Alpha: " + ALPHA, 0, 2);
//			Lab4.lcd.drawString("Beta: " + BETA, 0, 3);
//			Lab4.lcd.drawString("Final: " + FINAL_ANGLE, 0, 4);
		} else if(Lab4.isLightLocalizing) {
//			Lab4.lcd.clear();
//			Lab4.lcd.drawString("color: "+ LightLocalizer.newColor, 0, 2);
//			Lab4.lcd.drawString("x: "+ LightLocalizer.result[0], 0, 3);
//			Lab4.lcd.drawString("y: "+ LightLocalizer.result[1], 0, 4);
//			Lab4.lcd.drawString("theta: "+ LightLocalizer.result[2], 0, 5);
		}
		//else if(Lab4.isLightLocalizingTurn) {
//			Lab4.lcd.clear();
//			Lab4.lcd.drawString("passedLines: " + LightLocalizer.passedLine, 0, 1);
//			Lab4.lcd.drawString("points size: " + LightLocalizer.points.size(), 0, 2);
//			Lab4.lcd.drawString("x: " + odo.getXYT()[0], 0, 3);
//			Lab4.lcd.drawString("y: " + odo.getXYT()[1], 0, 4);
//			Lab4.lcd.drawString("theta: " + odo.getXYT()[2], 0, 5);

	//	}

	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}

	
}
