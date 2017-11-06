package NavigationTest;


import lejos.hardware.Button;	
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class NavigationTestMain {
	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 9.35;
	private static final Port usPort = LocalEV3.get().getPort("S1");
	public static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3LargeRegulatedMotor pulleyMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	static Port lsPortLeft = LocalEV3.get().getPort("S2");
	static Port lsPortRight = LocalEV3.get().getPort("S4");


	public static void main(String[] args){
		

		@SuppressWarnings("resource")
		SensorModes colorSensorLeft = new EV3ColorSensor(lsPortLeft);
		SampleProvider colorValueLeft = colorSensorLeft.getMode("Red");
		float[] colorDataLeft = new float[3];

		@SuppressWarnings("resource")
		SensorModes colorSensorRight = new EV3ColorSensor(lsPortRight);
		SampleProvider colorValueRight = colorSensorRight.getMode("Red");
		float[] colorDataRight = new float[3];
		
		int buttonChoice;
		
		final TextLCD t = LocalEV3.get().getTextLCD();
		@SuppressWarnings("resource")
		SensorModes usSensor = new EV3UltrasonicSensor(usPort);
		final SampleProvider usDistance = usSensor.getMode("Distance");
		float[] usData = new float[usDistance.sampleSize()];
		final Odometer odometer = new Odometer(leftMotor, rightMotor);
		final Navigation nav = new Navigation(colorValueLeft, colorDataLeft, colorValueRight, colorDataRight);
		final UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, nav);
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
		if(buttonChoice == Button.ID_LEFT || buttonChoice == Button.ID_RIGHT){


			leftMotor.forward();
			leftMotor.flt();
			rightMotor.forward();
			rightMotor.flt();
			
			odometer.start();
			odometryDisplay.start();
			
			
			usPoller.start();
			Thread move = new Thread() {
				/**
				 * Definition of the navigation thread
				 */
				public void run() {
					nav.travelTo(usDistance, odometer, WHEEL_RADIUS, WHEEL_RADIUS, TRACK, 2,
							2, usPoller);
				}

			};
			move.start();
			OdoCorrTest odometryCorrection = new OdoCorrTest(odometer, colorValueLeft, colorDataLeft
					, colorValueRight, colorDataRight, nav, usPoller, usDistance);
//			odometryCorrection.start();
			
		} while(Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
