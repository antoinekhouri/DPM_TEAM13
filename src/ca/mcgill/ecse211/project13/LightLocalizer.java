package ca.mcgill.ecse211.project13;

import lejos.hardware.Sound;		

import lejos.robotics.SampleProvider;
/**
 * This class implements the light localization part of the project. First the robot
 * moves until one of the sensors detects a grid line, then that sensor's side's motor
 * is stopped until the other sensor detects the line as well. Then the other side's motor
 * reverses for 2 cm, in order for the robot to be aligned with the grid. Then the robot
 * turns 90 degrees in the appropriate direction (based on the starting corner) and repeats
 * the same process as the in the previous direction to align again. 
 * @author Veronica Nasseem, Nusaiba Radi, Antoine Khouri, Nikki Daly
 *
 */
public class LightLocalizer {

	public static int ROTATION_SPEED = 40;
	public static int FORWARD_SPEED = 75;
	public static int ACCELERATION = 600;
	private static final double tileLength = 30.48;
	/* knowing that the normalized lightDensity is 0(darkest)-1.5(brightest), 
	 * we tested with different values and found that our sensor recognizes the black lines at 0.2 and below 
	 */
	private static double lightDensity = 0.40;
	private boolean isOkay = false;
	private int position;
	private Odometer odometer;
	private SampleProvider colorSensorLeft;
	private float[] colorDataLeft;
	private SampleProvider colorSensorRight;
	private float[] colorDataRight;
	private boolean shouldGoBackwards = false;
	private boolean isLeftSensor = false;
	private boolean isRightSensor = false;
	private boolean isDone = false;
	private double finalX;
	private double finalY;
	private double finalTheta;
	private boolean isZipLineLocalization = false;
	// private EV3LargeRegulatedMotor MainProject.leftMotor, MainProject.rightMotor;
	
	/**
	 * Default constructor
	 * @param odometer odometer used to poll 
	 * @param colorSensorLeft left color sensor of the robot
	 * @param colorDataLeft left color sensor's data buffer
	 * @param colorSensorRight right color sensor of the robot
	 * @param colorDataRight right color sensor's data buffer
	 * @param x x position the robot is localizing to
	 * @param y y position the robot is localizing to
	 * @param theta theta angle the robot is localizing to
	 */
	public LightLocalizer(Odometer odometer, SampleProvider colorSensorLeft, float[] colorDataLeft, 
			SampleProvider colorSensorRight, float[] colorDataRight, double x, double y, double theta) 
	{
		this.odometer = odometer;
		this.colorSensorLeft = colorSensorLeft;
		this.colorDataLeft = colorDataLeft;
		this.colorSensorRight = colorSensorRight;
		this.colorDataRight = colorDataRight;
		this.finalX = x;
		this.finalY = y;
		this.finalTheta = theta;
		//this.MainProject.leftMotor = MainProject.leftMotor;
		//this.MainProject.rightMotor = MainProject.rightMotor;

		MainProject.leftMotor.setAcceleration(ACCELERATION);
		MainProject.rightMotor.setAcceleration(ACCELERATION);

	}
	/**
	 * Secondary constructor used for post-zip line localization
	 * @param odometer odometer used to poll the robot's current position
	 * @param colorSensorLeft left color sensor
	 * @param colorDataLeft left color sensor's data buffer
	 * @param colorSensorRight right color sensor
	 * @param colorDataRight right color sensor's data buffer
	 */
	public LightLocalizer(Odometer odometer, SampleProvider colorSensorLeft, float[] colorDataLeft, 
			SampleProvider colorSensorRight, float[] colorDataRight) 
	{
		this.odometer = odometer;
		this.colorSensorLeft = colorSensorLeft;
		this.colorDataLeft = colorDataLeft;
		this.colorSensorRight = colorSensorRight;
		this.colorDataRight = colorDataRight;
		this.isZipLineLocalization = true;
	}
	//Localize robot using the light sensor

	/**
	 * This method implements the localization behaviour specified
	 * @param x x coordinate the robot is localizing to
	 * @param y y coordinate the robot is localizing to 
	 * @param theta theta angle the robot is localizing to
	 * @param isAfter boolean variable used to determined if this is a post-zipline 
	 * localization
	 */
	public void localize(double x, double y, double theta, boolean isAfter) {
		if(isAfter){
			MainProject.leftMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, 15), true);
			MainProject.rightMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, 15), false);
		}
		this.finalX = x;
		this.finalY = y;
		this.finalTheta = theta;
		//	}
		// set the speeds of the motors and move forward (in the y direction)
		MainProject.leftMotor.setSpeed(FORWARD_SPEED);
		MainProject.rightMotor.setSpeed(FORWARD_SPEED);
		MainProject.leftMotor.forward();
		MainProject.rightMotor.forward();

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
			MainProject.leftMotor.stop(true);
			isLeftSensor = true;
			Sound.beep();
		}
		else if(getColorDataRight()< lightDensity){

			MainProject.rightMotor.stop(true);			
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
			MainProject.rightMotor.stop(false);
			if(!isAfter) {
				MainProject.rightMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, -1.5), false);
			}else {
				MainProject.rightMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, -1), false);
			}
			MainProject.rightMotor.stop(false);
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
			MainProject.leftMotor.stop(false);
			if(!isAfter) {
				MainProject.leftMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, -1.5), false);
			}else {
				MainProject.leftMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, -1), false);
			}

			MainProject.leftMotor.stop(false);
			isLeftSensor = true;
		}
		if(isRightSensor && isLeftSensor && !isAfter){
			MainProject.leftMotor.rotate(convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, 90.0), true);
			MainProject.rightMotor.rotate(-convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, 90.0), false);
			isLeftSensor = false;
			isRightSensor = false;
		}else{
			if (isRightSensor && isLeftSensor){
				isLeftSensor = false;
				isRightSensor = false;
			}
		}


		// set the speeds of the motors and move forward (in the x direction)
		if(!isAfter){
			MainProject.leftMotor.setSpeed(FORWARD_SPEED);
			MainProject.rightMotor.setSpeed(FORWARD_SPEED);
			MainProject.leftMotor.forward();
			MainProject.rightMotor.forward();

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
				MainProject.leftMotor.stop(true);
				isLeftSensor = true;
				Sound.beep();
			}
			else if(getColorDataRight()< lightDensity){
				MainProject.rightMotor.stop(true);
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
				MainProject.rightMotor.stop(false);
				MainProject.rightMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, -0.5), false);
				MainProject.rightMotor.stop(false);
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
				MainProject.leftMotor.stop(false);
				MainProject.leftMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, -0.5), false);
				MainProject.leftMotor.stop(false);
				isLeftSensor = true;
				Sound.beep();
			}
		}
		else{
			isRightSensor = true;
			isLeftSensor = true;
		}
		if(!(finalX==7 && finalY==1)){
			if(isRightSensor && isLeftSensor){
				MainProject.leftMotor.setSpeed(ROTATION_SPEED);
				MainProject.rightMotor.setSpeed(ROTATION_SPEED);
				if(theta==0 && !isAfter){

					MainProject.leftMotor.rotate(convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, -90.0), true);
					MainProject.rightMotor.rotate(-convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, -90.0), false);
					isOkay = false;
				}
				else if(theta==180 && finalX==7){
					//					MainProject.leftMotor.backward();
					//					MainProject.rightMotor.forward();
					MainProject.leftMotor.rotate(convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, -90.0), true);
					MainProject.rightMotor.rotate(-convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, -90.0), false);
					isOkay = false;
				}else if(theta !=0 && theta!=180){
					MainProject.leftMotor.stop(false);
					MainProject.rightMotor.stop(false);
					isOkay = false;
				}
				if (isAfter){
					MainProject.leftMotor.rotate(convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, 90.0), true);
					MainProject.rightMotor.rotate(-convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, 90.0), false);
					isOkay = false;
				}
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e1) {

					e1.printStackTrace();
				}
				if(isOkay){
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
						MainProject.leftMotor.stop(false);
						isLeftSensor = true;
						Sound.beep();
						while(getColorDataRight()>lightDensity){
							try {
								Thread.sleep(1);
							} catch (InterruptedException e) {
								//Auto-generated catch block
								e.printStackTrace();
							}
						}
						Sound.beep();
						//					MainProject.rightMotor.stop(false);
						//					MainProject.rightMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, -1.5), false);
						MainProject.rightMotor.stop(false);
					}
					else if(getColorDataRight()< lightDensity){
						MainProject.rightMotor.stop(false);
						isRightSensor = true;
						Sound.beep();
						while(getColorDataLeft()>lightDensity){
							try {
								Thread.sleep(1);
							} catch (InterruptedException e) {
								//Auto-generated catch block
								e.printStackTrace();
							}
						}
						Sound.beep();
						//					MainProject.leftMotor.stop(false);
						//					MainProject.leftMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, -1.5), false);
						MainProject.leftMotor.stop(false);
					}
				}
			}
			
		}

		odometer.setX(x*tileLength);
		odometer.setY(y*tileLength);
		odometer.setTheta(finalTheta);

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