package NavigationTest;


import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;


public class OdoCorrTest extends Thread{


	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;

	//initializing color/light sensor


	private static double lightDensity = 0.40;
	//counter for black lines (along x and y axes)
	int xCounter = 0; 
	int yCounter = 0;
	double tileWidth = 30.48;

	private boolean isLeftSensor = false;
	private boolean isRightSensor = false;
	private SampleProvider colorSensorLeft;
	private float[] colorDataLeft;
	private SampleProvider colorSensorRight;
	private float[] colorDataRight;
	private Navigation nav;
	private SampleProvider usDistance;
	private UltrasonicPoller usPoller;

	// constructor
	public OdoCorrTest(Odometer odometer,  SampleProvider colorSensorLeft, float[] colorDataLeft, 
			SampleProvider colorSensorRight, float[] colorDataRight, Navigation nav, UltrasonicPoller usPoller
			, SampleProvider usDistance) {
		this.odometer = odometer;
		this.colorSensorLeft = colorSensorLeft;
		this.colorDataLeft = colorDataLeft;
		this.colorSensorRight = colorSensorRight;
		this.colorDataRight = colorDataRight;
		this.nav = nav;
		this.usPoller = usPoller;
		this.usDistance = usDistance;
	}

	// run method (required for Thread)
	public void run() {

		while(true){
			if(odometer.getX()>10 || odometer.getY()>10){
				while (!nav.getIsTurning()) {


					//after testing, found that sensor recognizes the black lines when the light intensity is less than 0.4
					if(getColorDataLeft() < lightDensity ){
						NavigationTestMain.leftMotor.stop(true);					
						isLeftSensor = true;
						Sound.beep();
					}
					else if(getColorDataRight()< lightDensity){
						NavigationTestMain.rightMotor.stop(true);
						isRightSensor = true;
						Sound.beep();
					}
					if(isLeftSensor){
						while(getColorDataRight() > lightDensity){
							try {
								Thread.sleep(1);
							} catch (InterruptedException e) {
								//Auto-generated catch block
								e.printStackTrace();
							}
						}
						Sound.beep();
						NavigationTestMain.rightMotor.stop(true);
						isRightSensor = true;
					}
					else if(isRightSensor){
						while(getColorDataLeft() > lightDensity){
							try {
								Thread.sleep(1);
							} catch (InterruptedException e) {
								//Auto-generated catch block
								e.printStackTrace();
							}
						}
						Sound.beep();
						NavigationTestMain.leftMotor.stop(true);
						isLeftSensor = true;
					}
					if(isLeftSensor && isRightSensor){
						//				odometer.setX(xCounter*tileWidth);
						//				odometer.setY(yCounter*tileWidth);
//						NavigationTestMain.rightMotor.setSpeed(150);
//						NavigationTestMain.leftMotor.setSpeed(150);
//						NavigationTestMain.rightMotor.forward();
//						NavigationTestMain.leftMotor.forward();
						nav.travelTo(usDistance, odometer, NavigationTestMain.WHEEL_RADIUS, 
								NavigationTestMain.WHEEL_RADIUS, NavigationTestMain.TRACK, 2,
							3, usPoller);
						isLeftSensor = false;
						isRightSensor = false;
						if(Math.abs(odometer.getTheta())<10 ){
							yCounter++;
						}
						else if(Math.abs(odometer.getTheta()-90)<10){
							xCounter++;
						}
						else if(Math.abs(odometer.getTheta()-180)<10){
							yCounter--;
						}
						else{
							xCounter--;
						}
					}

					// this ensure the odometry correction occurs only once every period



					// there is nothing to be done here because it is not
					// expected that the odometry correction will be
					// interrupted by another thread
				}
			}	
		}
	}
	private float getColorDataLeft() {
		colorSensorLeft.fetchSample(colorDataLeft, 0);
		float colorBrightnessLevel = (colorDataLeft[0] + colorDataLeft[1] + colorDataLeft[2]);
		return colorBrightnessLevel;
	}
	private float getColorDataRight(){
		colorSensorRight.fetchSample(colorDataRight, 0);
		float colorBrightnessLevel = (colorDataRight[0] + colorDataRight[1] + colorDataRight[2]);
		return colorBrightnessLevel;
	}

}
