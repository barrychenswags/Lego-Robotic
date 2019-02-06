// Lab2.java
package ca.mcgill.ecse211.lab4;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
/**
 * @author Ahmed Elehwany 260707540, Barry Chen
 * This the the main class where we 
 * 1. Start the program,
 * 2. Start the UI interface selection between rising edge and falling edge.
 * 3. Instantiates Ultrasonic localizer then light localizer.
 *
 */
public class Lab4 {
	
	public static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.15;
	public static final double SQUARE_SIZE = 30.48;
	public static final double TRACK = 13.7;
	public static boolean isUSLocalizing = false;
	public static boolean isLightLocalizing = false;
	public static boolean isLightLocalizingTurn = false;
	static Odometer odometer = null;

	// Motor Objects, and Robot related parameters
	static final Port usPort = LocalEV3.get().getPort("S1");
	static final Port portColor = LocalEV3.get().getPort("S4");
	public static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  
	
	/**
	 * This is the main method for this class where the program starts
	 * @param args
	 * @throws Exception
	 */
  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice;

    // Odometer related objects
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);  
    Display odometryDisplay = new Display(lcd); 
    //ObstacleAvoidance obstacleavoidance = new ObstacleAvoidance(leftMotor, rightMotor, TRACK, WHEEL_RAD);
    do {
		lcd.clear();   		// clear the display

		// Ask the user whether map 1 or 2 / map 3 or 4 should be selected
		lcd.drawString("       |       ", 0, 0);
		lcd.drawString("Falling|Rising ", 0, 1);
		lcd.drawString(" Edge  |  Edge ", 0, 2);
		lcd.drawString("       |       ", 0, 3);
		lcd.drawString("       |       ", 0, 4);

		buttonChoice = Button.waitForAnyPress();      // Record choice (left or right press)

		// Until button pressed
	} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
    
       // Start odometer and display threads
          Thread odoThread = new Thread(odometer);
          odoThread.start();
          Thread odoDisplayThread = new Thread(odometryDisplay);
          odoDisplayThread.start();
          Navigation nav = new Navigation(leftMotor, rightMotor, TRACK, WHEEL_RAD);
          
          if (buttonChoice == Button.ID_LEFT) {
  			isUSLocalizing = true;
  			UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer( odometer, nav);
  			usLocalizer.fallingEdge();
  		}
  		else {
  			isUSLocalizing = true;
  			UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(odometer, nav);
  			usLocalizer.risingEdge();
  		}
          isUSLocalizing = false;
  		// Wait before starting
  		Button.waitForAnyPress();
  		
  		 if (buttonChoice == Button.ID_RIGHT || buttonChoice == Button.ID_LEFT) {   
  			 isLightLocalizing = true;
  	    	 LightLocalizer lightLocalizer  = new LightLocalizer(odometer, nav);     // Set map 2
  	    	 lightLocalizer.start();
  	      }
        
      }
    }