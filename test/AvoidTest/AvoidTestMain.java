package AvoidTest;

import NavigationTest.Navigation;
import NavigationTest.Odometer;
import NavigationTest.OdometryDisplay;
import ca.mcgill.ecse211.project13.MainProject;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class AvoidTestMain {

	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	// the motor to rotate the sensor
	public static final EV3LargeRegulatedMotor sMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

	private static final Port usPort = LocalEV3.get().getPort("S1");

	public static final double WHEEL_RADIUS = 2.2;
	public static final double TRACK = 14.3;

	// the coordinate for the next way point
	public static double nextX = 0;
	public static double nextY = 0;

	private static int[] p1 = { 2, 1 };
	private static int[] p2 = { 1, 1 };
	private static int[] p3 = { 1, 2 };
	private static int[] p4 = { 2, 0 };
	private static int[][] path = { p1, p2, p3, p4 };

	public static void main(String[] args) {

		final TextLCD t = LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(leftMotor, rightMotor);
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t);
		final Navigation navigation = new Navigation();
		@SuppressWarnings("resource")
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
		SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
																	// this instance
		float[] usData = new float[usDistance.sampleSize()];
		UltrasonicPoller uspoller = new UltrasonicPoller(usDistance, usData);
		ObstacleAvoidTest avoid = new ObstacleAvoidTest(navigation, odometer, uspoller);

		do {
			// clear the display
			t.clear();

			odometer.start();
			odometryDisplay.start();
			uspoller.start();
			avoid.start();
			for (int i = 0; i < 5; i++) {
					nextX = path[i][0];
					nextY = path[i][1];
					// store the next way point
					navigation.travelTo(usDistance, odometer, (double)MainProject.WHEEL_RADIUS, (double)MainProject.WHEEL_RADIUS, (double)MainProject.TRACK, (double)MainProject.X0_final,
							(double)MainProject.Y0_final);
				

			}

		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}
}


//package AvoidTest;
//
//
//import lejos.hardware.sensor.*;
//import lejos.hardware.ev3.LocalEV3;
//import lejos.hardware.motor.EV3LargeRegulatedMotor;
//import lejos.hardware.port.Port;
//import lejos.robotics.SampleProvider;
//import lejos.hardware.Button;
//
//
//
//
//public class AvoidTestMain {
//
//
//	  // Parameters: adjust these for desired performance
//
//	  private static final int bandCenter = 25; // Offset from the wall (cm)
//	  private static final int bandWidth = 3; // Width of dead band (cm)
//	  private static final int motorLow = 100; // Speed of slower rotating wheel (deg/sec)
//	  private static final int motorHigh = 200; // Speed of the faster rotating wheel (deg/seec)
//
//
//	  private static final Port usPort = LocalEV3.get().getPort("S1");
//	  //private static final Port tPort = LocalEV3.get().getPort("S4");
//	  public static final EV3LargeRegulatedMotor leftMotor =
//	      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
//	  public static final EV3LargeRegulatedMotor rightMotor =
//	      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
//
//	  // Main entry point - instantiate objects used and set up sensor
//
//	  public static void main(String[] args) {
//
//	    int option = 0;
//	    Printer.printMainMenu(); // Set up the display on the EV3 screen
//	    while (option == 0) // and wait for a button press. The button
//	      option = Button.waitForAnyPress(); // ID (option) determines what type of control to use
//
//	    // Setup controller objects
//
////	    BangBangController bangbangController =
////	        new BangBangController(bandCenter, bandWidth, motorLow, motorHigh);
//
//	    PControllerTest pController = new PControllerTest(bandCenter, bandWidth);
//
//	    // Setup ultrasonic sensor
//	    // There are 4 steps involved:
//	    // 1. Create a port object attached to a physical port (done already above)
//	    // 2. Create a sensor instance and attach to port
//	    // 3. Create a sample provider instance for the above and initialize operating mode
//	    // 4. Create a buffer for the sensor data
//
//	    @SuppressWarnings("resource") // Because we don't bother to close this resource
//	    SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
//	    //@SuppressWarnings("resource")
//		//SensorModes tSensor = new EV3UltrasonicSensor(tPort);
//	    
//	    SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from this instance
//	    //SampleProvider tDistance = tSensor.getMode(0);
//	    
//	    float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are returned
//	    //float[] tData = new float[tDistance.sampleSize()];                                                    
//
//	    // Setup Printer
//	    // This thread prints status information in the background
//	    Printer printer = null;
//
//	    // Setup Ultrasonic Poller // This thread samples the US and invokes
//	    UltrasonicPoller usPoller = null; // the selected controller on each cycle
//
//	    // Depending on which button was pressed, invoke the US poller and printer with the
//	    // appropriate constructor.
//
//	    switch (option) {
////	      case Button.ID_LEFT: // Bang-bang control selected
////	        usPoller = new UltrasonicPoller(usDistance, usData, bangbangController);
////	        printer = new Printer(option, bangbangController);
////	        break;
//	      case Button.ID_RIGHT: // Proportional control selected
//	        usPoller = new UltrasonicPoller(usDistance, usData, pController);
//	        printer = new Printer(option, pController);
//	        break;
//	      default:
//	        System.out.println("Error - invalid button"); // None of the above - abort
//	        System.exit(-1);
//	        break;
//	    }
//
//	    // Start the poller and printer threads
//	    usPoller.start();
//	    printer.start();
//
//	    // Wait here forever until button pressed to terminate wallfollower
//	    Button.waitForAnyPress();
//	    System.exit(0);
//
//	  }
//	
//
//}
