package NavigationTest;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class NavigationTestMain {
	  public static final double WHEEL_RADIUS = 2.1;
	  public static final double TRACK = 9.7;
	  private static final Port usPort = LocalEV3.get().getPort("S1");
	  private static final Port lsPort = LocalEV3.get().getPort("S2");
	  public static final EV3LargeRegulatedMotor leftMotor =
	      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	  public static final EV3LargeRegulatedMotor rightMotor =
	      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	  public static final EV3LargeRegulatedMotor pulleyMotor =
	      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	  
	  public static void main(String[] args){
			 	
		  		int buttonChoice;

			    final TextLCD t = LocalEV3.get().getTextLCD();
			    @SuppressWarnings("resource")
				SensorModes usSensor = new EV3UltrasonicSensor(usPort);
				final SampleProvider usDistance = usSensor.getMode("Distance");
				float[] usData = new float[usDistance.sampleSize()];
			    final Odometer odometer = new Odometer(leftMotor, rightMotor);
			    OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t);
			    
			    

			    do {
			      // clear the display
			      t.clear();


			      t.drawString("< Left ", 0, 0);
			      t.drawString(" start ", 0, 1);


			      buttonChoice = Button.waitForAnyPress();
			    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT && buttonChoice!=
			    		Button.ID_ESCAPE);
			    if(buttonChoice == Button.ID_ESCAPE){
			    	System.exit(0);
			    }
	  }
}
