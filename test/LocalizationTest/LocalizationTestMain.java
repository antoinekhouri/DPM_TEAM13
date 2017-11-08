package LocalizationTest;



import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LocalizationTestMain {
	  public static final double WHEEL_RADIUS = 2.1;
	  public static final double TRACK = 9.1;
	  private static final Port usPort = LocalEV3.get().getPort("S3");
	  private static final Port lsPortLeft = LocalEV3.get().getPort("S2");
	  private static final Port lsPortRight = LocalEV3.get().getPort("S4");
	  public static final EV3LargeRegulatedMotor leftMotor =
	      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	  public static final EV3LargeRegulatedMotor rightMotor =
	      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	  
	  public static void main(String[] args){
		  
		  	
		  	int buttonChoice;
		    final TextLCD t = LocalEV3.get().getTextLCD();
		    final Odometer odometer = new Odometer(leftMotor, rightMotor);
		    OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t);
		    
		    @SuppressWarnings("resource")
		    SensorModes usSensor = new EV3UltrasonicSensor(usPort);
		    final SampleProvider usDistance = usSensor.getMode("Distance");
		    float[] usData = new float[usDistance.sampleSize()];

		    @SuppressWarnings("resource")
		    SensorModes colorSensorLeft = new EV3ColorSensor(lsPortLeft);
		    SampleProvider colorValueLeft = colorSensorLeft.getMode("Red");
		    float[] colorDataLeft = new float[3];
		    
		    @SuppressWarnings("resource")
			SensorModes colorSensorRight = new EV3ColorSensor(lsPortRight);
		    SampleProvider colorValueRight = colorSensorRight.getMode("Red");
		    float[] colorDataRight = new float[3];
		    
		    do {
		        t.clear();
		        t.drawString(" < Left   |   Right >", 0, 0);
		        t.drawString(" Falling  |   Rising ", 0, 1);
		        t.drawString(" Edge     |   Edge   ", 0, 2);

		        buttonChoice = Button.waitForAnyPress();
		    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT && buttonChoice
		    		!= Button.ID_ESCAPE);
		    if (buttonChoice == Button.ID_LEFT) {
				USLocalizationTest usLocalizer = new USLocalizationTest( true,odometer);
				UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, usLocalizer);
				usPoller.start();
				odometer.start();
				odometryDisplay.start();
				Button.waitForAnyPress();	
				if (buttonChoice == Button.ID_ESCAPE) {
			        System.exit(0);
			    }
				LightLocalizationTest lsTest = new LightLocalizationTest(odometer, colorValueLeft, colorDataLeft
						, colorValueRight, colorDataRight);
				lsTest.localize();
		    } else if (buttonChoice == Button.ID_RIGHT) {
				USLocalizationTest usLocalizer = new USLocalizationTest( false,odometer);
				UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, usLocalizer);
				usPoller.start();
				odometer.start();
				odometryDisplay.start();				
				Button.waitForAnyPress();
				if (buttonChoice == Button.ID_ESCAPE) {
			        System.exit(0);
			    }
				LightLocalizationTest lsTest = new LightLocalizationTest(odometer, colorValueLeft, colorDataLeft,
						colorValueRight, colorDataRight);
				lsTest.localize();
		    } else if (buttonChoice == Button.ID_ESCAPE) {
		        System.exit(0);
		    }while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		    System.exit(0);
	  }
}
