package ZipLineTest;

import ZipLineTest.Odometer;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class ZiplineTestMain {
	public static final double WHEEL_RADIUS = 2.1;
	  public static final double TRACK = 9.7;
	  public static final EV3LargeRegulatedMotor leftMotor =
	      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	  public static final EV3LargeRegulatedMotor rightMotor =
	      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	  public static final EV3LargeRegulatedMotor pulleyMotor =
	      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));

	  public static void main(String[] args) throws InterruptedException {


	    int buttonChoice;

	    final Odometer odometer = new Odometer(leftMotor, rightMotor);

	    final Zipline trav = new Zipline(odometer);
	    
	    // create thread for traversing the zipline
	    Thread traverse = new Thread() {
	      public void run() {
	        trav.traverse();
	      }
	    };

	    // if escape is pressed, the program will stop
	    buttonChoice = Button.waitForAnyPress();
	    if (buttonChoice == Button.ID_ESCAPE) {
	      System.exit(0);
	    }
	    
	    // start traversing zipline once the robot reaches the right coordinates
	    traverse.start();
	    traverse.join();

	    // escape program when escape button is pressed
	    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
	    System.exit(0);
	  }
}
