package SearchFlag;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class SearchFlagMainTest {
	
	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 9.40;
	private static final Port usPort = LocalEV3.get().getPort("S3");
	private static final Port lsPortLeft = LocalEV3.get().getPort("S1");
	private static final Port lsPortRight = LocalEV3.get().getPort("S2");
	private static final Port lsFront = LocalEV3.get().getPort("S4");
	public static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3LargeRegulatedMotor pulleyMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	// the motor to rotate the sensor
	public static final EV3LargeRegulatedMotor sMotor = 
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

	
	public static void main(String[] args) {



	int zipLineEndX=0;
	int zipLineEndY=0;
	
	//lower left
	int searchZoneX=0;
	int searchZoneY=0;
	//upperright
	int searchZoneX2=0;
	int searchZoneY2=0;
	boolean isZipLineVertical = false;
	
	final TextLCD t = LocalEV3.get().getTextLCD();
	final Odometer odometer = new Odometer(leftMotor, rightMotor);
	//		OdometryCorrection odometryCorrection = new OdometryCorrection(odometer);
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
	
	@SuppressWarnings("resource")
	SensorModes colorSensorFront = new EV3ColorSensor(lsFront);
	SampleProvider colorValueFront = colorSensorRight.getMode("Red");
	float[] colorDataFront = new float[3];
	
	UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(true, odometer);
	final UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, usLocalizer);

			final Navigation nav2 = new Navigation(colorValueLeft, colorDataLeft, colorValueRight, colorDataRight);
			if(!isZipLineVertical){
				nav2.travelTo(usDistance, odometer, WHEEL_RADIUS, WHEEL_RADIUS, TRACK,searchZoneX, searchZoneY, zipLineEndX, zipLineEndY, usPoller, isZipLineVertical);
			}else{
				nav2.travelTo(usDistance, odometer, WHEEL_RADIUS, WHEEL_RADIUS, TRACK,searchZoneX, searchZoneY, zipLineEndX, zipLineEndY, usPoller);
			}
			
			final SearchFlagDetectTest search = new SearchFlagDetectTest(odometer, colorSensorFront, colorDataFront);
			search.detect(searchZoneX, searchZoneY, searchZoneX2, searchZoneY2, odometer);
	}
}

