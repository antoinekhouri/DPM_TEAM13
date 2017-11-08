package LocalizationTest;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class LightLocalizationTest {

	public static int ROTATION_SPEED = 40;
	public static int FORWARD_SPEED = 75;
	public static int ACCELERATION = 600;
	private static final double tileLength = 30.48;
	/* knowing that the normalized lightDensity is 0(darkest)-1(brightest), 
	 * we tested with different values and found that our sensor recognizes the black lines at 0.2 and below 
	 */
	private static double lightDensity = 0.40;

	private Odometer odometer;
	private SampleProvider colorSensorLeft;
	private float[] colorDataLeft;
	private SampleProvider colorSensorRight;
	private float[] colorDataRight;
	private boolean shouldGoBackwards = false;
	private boolean isLeftSensor = false;
	private boolean isRightSensor = false;
	private boolean isDone = false;
	private int position;
	private int finalX;
	private int finalY;
	private int finalTheta;
	// private EV3LargeRegulatedMotor LocalizationTestMain.leftMotor, LocalizationTestMain.rightMotor;
	/**
	 * Default constructor
	 * @param odometer Odometer used to get information about robot's current position
	 * @param colorSensor	SampleProvider used by the light sensor
	 * @param colorData Buffer used to store the light sensor's data
	 */
	public LightLocalizationTest(Odometer odometer, SampleProvider colorSensorLeft, float[] colorDataLeft, 
			SampleProvider colorSensorRight, float[] colorDataRight)
	//EV3LargeRegulatedMotor LocalizationTestMain.leftMotor, EV3LargeRegulatedMotor LocalizationTestMain.rightMotor) 
	{
		this.odometer = odometer;
		this.colorSensorLeft = colorSensorLeft;
		this.colorDataLeft = colorDataLeft;
		this.colorSensorRight = colorSensorRight;
		this.colorDataRight = colorDataRight;
		//this.LocalizationTestMain.leftMotor = LocalizationTestMain.leftMotor;
		//this.LocalizationTestMain.rightMotor = LocalizationTestMain.rightMotor;

		LocalizationTestMain.leftMotor.setAcceleration(ACCELERATION);
		LocalizationTestMain.rightMotor.setAcceleration(ACCELERATION);

	}

	//Localize robot using the light sensor
	/**
	 * Localizes the robot the the desired grid line intersection
	 * @param position 0-1-2-3, the position on the grid where the robot is placed at
	 */
	public void localize() {
		
		//	}
		// set the speeds of the motors and move forward (in the y direction)
		LocalizationTestMain.leftMotor.setSpeed(FORWARD_SPEED);
		LocalizationTestMain.rightMotor.setSpeed(FORWARD_SPEED);
		LocalizationTestMain.leftMotor.forward();
		LocalizationTestMain.rightMotor.forward();

		//keep going forward until we hit a black line
		while (getColorDataLeft() > lightDensity && getColorDataRight() > lightDensity) {
			//try-catch from ultrasonic poller
			try {
				Thread.sleep(1);
			} catch (InterruptedException e) {
				//Auto-generated catch block
				e.printStackTrace();
			}
		}

		//reached a black line so stop
		if(getColorDataLeft() < lightDensity ){
			LocalizationTestMain.leftMotor.stop(true);
			isLeftSensor = true;
			Sound.beep();
		}
		else if(getColorDataRight()< lightDensity){
			LocalizationTestMain.rightMotor.stop(true);
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
			LocalizationTestMain.rightMotor.stop(true);
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
			LocalizationTestMain.leftMotor.stop(true);
			isLeftSensor = true;
		}
		if(isRightSensor && isLeftSensor){
			LocalizationTestMain.leftMotor.rotate(convertAngle(LocalizationTestMain.WHEEL_RADIUS, LocalizationTestMain.TRACK, 90.0), true);
			LocalizationTestMain.rightMotor.rotate(-convertAngle(LocalizationTestMain.WHEEL_RADIUS, LocalizationTestMain.TRACK, 90.0), false);
			isLeftSensor = false;
			isRightSensor = false;
		}


		// set the speeds of the motors and move forward (in the x direction)
		LocalizationTestMain.leftMotor.setSpeed(FORWARD_SPEED);
		LocalizationTestMain.rightMotor.setSpeed(FORWARD_SPEED);
		LocalizationTestMain.leftMotor.forward();
		LocalizationTestMain.rightMotor.forward();

		// keep going forward until we hit a black line
		while (getColorDataLeft() >  lightDensity && getColorDataRight() > lightDensity) { 
			//try-catch from ultrasonic poller

			try {
				Thread.sleep(1);
			} catch (InterruptedException e) {
				//Auto-generated catch block
				e.printStackTrace();
			}
		}

		//reached a black line so stop
		if(getColorDataLeft() < lightDensity ){
			LocalizationTestMain.leftMotor.stop(true);
			isLeftSensor = true;
			Sound.beep();
		}
		else if(getColorDataRight()< lightDensity){
			LocalizationTestMain.rightMotor.stop(true);
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
			LocalizationTestMain.rightMotor.stop(true);
			isRightSensor = true;
			Sound.beep();
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
			LocalizationTestMain.leftMotor.stop(true);
			isLeftSensor = true;
			Sound.beep();
		}


		odometer.setX(0);
		odometer.setY(0);
		odometer.setTheta(0);

	}

	// conversion methods
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	// gets the data from the color sensor, and returns a value corresponding
	// to the overall "brightness" which is the avg of magnitudes of red, green, and blue
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