package ca.mcgill.ecse211.project13;




import LocalizationTest.LocalizationTestMain;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;



public class Navigation implements UltrasonicController {
	// Constants and variables
	private static final int FORWARD_SPEED = 150;
	private static final int FORWARD_SPEED_Right = 150;
	private static final int ROTATE_SPEED = 50;
	private static final double tileLength = 30.48;
	private double minDistance = 0;
	private Odometer odometer;


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
	private static double lightDensity = 0.40;

	private static final double pulley_to_roborCenter = 1.3;
	private boolean stop = false;
	private  boolean isTurning;
	private boolean isReady= false;
	private boolean isDoneWithX = false;
	private double xCounter;
	private double yCounter;
	private boolean isDoneWithY = false;
	/** 
	 * This constructor is simply used in order to indicate to the system whether or not it will
	 * require wall avoidance.
	 * @param isAvoidingWall a boolean that indicates whether or not the system will require to avoid a wall
	 * @return nothing
	 */
	public Navigation(SampleProvider colorSensorLeft, float[] colorDataLeft, SampleProvider colorSensorRight
			, float[] colorDataRight) {
		this.isTurning = false;
		this.colorSensorLeft = colorSensorLeft;
		this.colorDataLeft = colorDataLeft;
		this.colorSensorRight = colorSensorRight;
		this.colorDataRight = colorDataRight;
	}
	/**
	 * This method calculates the minimum angle to turn in order to face the destination,
	 * then uses the turnTo() method in order to turn to said angle, then calculates the distance required
	 * in order to arrive to the destination and finally it travels that distance
	 * @param usDistance SampleProvider used by the ultrasonic poller
	 * @param odometer Odometer used to get info about the current position and orientation of the robot
	 * @param leftRadius Double representing the left wheel radius, used for the wheel movement
	 * @param rightRadius Double representing the right wheel radius, used for the wheel movement
	 * @param width Double representing the distance between the two wheels, used for the robot movement
	 * @param x0 destination X coordinate
	 * @param y0 destination Y coordinate
	 * @param xC zip line start X coordinate
	 * @param yC zip line start y coordinate
	 */
	public void travelTo(SampleProvider usDistance, Odometer odometer, double leftRadius,
			double rightRadius, double width, double x0, double y0,double originalX, double originalY, UltrasonicPoller uspoller) {
		double xDistance = x0 * tileLength - odometer.getX();
		double yDistance = y0 * tileLength - odometer.getY();
		this.xCounter = originalX;
		this.yCounter = originalY;
		while(!isDoneWithY){

			MainProject.leftMotor.setSpeed(ROTATE_SPEED);
			MainProject.rightMotor.setSpeed(ROTATE_SPEED);
			if(!isDoneWithX){
				if (Math.abs(xDistance) < 10) { // If the destination is on the same X axis, do not rotate
					xDistance = 0;
				} else if (x0 * tileLength > odometer.getX()) { // If the destination is to the right of the
					// current position, rotate 90 degrees clockwise
					isTurning  = true;
					turnTo(90, odometer, leftRadius, rightRadius, width);
					isTurning = false;

				} else if (x0 * tileLength < odometer.getX()) { // Otherwise, rotate 90 degrees counter clock
					// wise
					isTurning = true;
					turnTo(270, odometer, leftRadius, rightRadius, width);
					isTurning = false;
				}
			}else if(isDoneWithX){
				if (Math.abs(yDistance) < 10) {
					yDistance = 0;
				} else if (y0 * tileLength > odometer.getY()) {
					isTurning = true;
					turnTo(0, odometer, leftRadius, rightRadius, width);
					isTurning = false;
				} else if (y0 * tileLength < odometer.getY()) {
					isTurning = true;
					turnTo(180, odometer, leftRadius, rightRadius, width);
					isTurning = false;
				}
			}

			MainProject.leftMotor.setSpeed(FORWARD_SPEED);
			MainProject.rightMotor.setSpeed(FORWARD_SPEED_Right);

			MainProject.leftMotor.forward();
			MainProject.rightMotor.forward();

			try {
				Thread.sleep(2000);
			} catch (InterruptedException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}
			while(getColorDataLeft() > lightDensity && getColorDataRight() > lightDensity){
				try {
					Thread.sleep(1);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}



			if(getColorDataLeft() < lightDensity ){
				//              MainProject.leftMotor.setSpeed(0);                   
				isLeftSensor = true;
				Sound.beep();
				//                while(getColorDataRight() > lightDensity){
				//                    try {
				//                        Thread.sleep(1);
				//                    } catch (InterruptedException e) {
				//                        // TODO Auto-generated catch block
				//                        e.printStackTrace();
				//                    }
				//                }
				Sound.beep();
				//              MainProject.rightMotor.setSpeed(0);
				isRightSensor = true;
			}
			else if(getColorDataRight()< lightDensity){
				//              MainProject.rightMotor.setSpeed(0);
				isRightSensor = true;
				Sound.beep();
				//                while(getColorDataLeft() > lightDensity){
				//                    try {
				//                        Thread.sleep(1);
				//                    } catch (InterruptedException e) {
				//                        // TODO Auto-generated catch block
				//                        e.printStackTrace();
				//                    }
				//                }
				Sound.beep();
				//              MainProject.leftMotor.setSpeed(0);
				isLeftSensor = true;
			}

			//          }
			if(isLeftSensor || isRightSensor){
				isReady =true;
				isLeftSensor = false;
				isRightSensor =false;
			}
			if(isReady){
				MainProject.leftMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, -2), true);
				MainProject.rightMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, -2), false);
				
				
				
				
				if(xCounter==x0){
					isDoneWithX = true;
				}
				if(!isDoneWithX){
					if(Math.abs(odometer.getTheta()-90)<10){
						xCounter++;
					}
					else if(Math.abs(odometer.getTheta()-270)<10){
						xCounter--;
					}
					odometer.setX(xCounter*tileLength);
					if(xCounter==x0){
						isDoneWithX = true;
					}
				}
				if (isDoneWithX && !isDoneWithY){
					if(Math.abs(odometer.getTheta())<10 || Math.abs(odometer.getTheta())>350){
						yCounter++;
					}else if(Math.abs(odometer.getTheta()-180)<10){
						yCounter--;
					}
					odometer.setY(yCounter*tileLength);
					if(yCounter==y0){
						isDoneWithY = true;
						MainProject.rightMotor.stop(true);
						MainProject.leftMotor.stop(true);
					}
				}


			}


		}




	}

	/**
	 * 
	 * @param theta the angle the robot should turn to
	 * @param odometer the odometer used to get information about robot's current position/orientation
	 * @param leftRadius Double representing the left wheel radius, used for the wheel movement
	 * @param rightRadius Double representing the right wheel radius, used for the wheel movement
	 * @param width Double representing the distance between the two wheels, used for the robot movement
	 */
	// Helper method to turn the robot to a given angle
	public void turnTo(double theta, Odometer odometer, double leftRadius, double rightRadius,
			double width) {

		MainProject.leftMotor.setSpeed(ROTATE_SPEED);
		MainProject.rightMotor.setSpeed(ROTATE_SPEED);
		while (theta - odometer.getTheta() > 360) {
			theta = theta - 360;
		}
		while (theta - odometer.getTheta() < 0) {
			theta = theta + 360;
		}
		if ((theta - odometer.getTheta()) > 180) {

			MainProject.rightMotor
			.rotate(convertAngle(rightRadius, width, 360 - (theta - odometer.getTheta())), true);
			MainProject.leftMotor
			.rotate((-convertAngle(leftRadius, width, 360 - (theta - odometer.getTheta()))), false);
		} else {
			MainProject.leftMotor.rotate((convertAngle(leftRadius, width, theta - odometer.getTheta())),
					true);
			MainProject.rightMotor.rotate(-convertAngle(rightRadius, width, theta - odometer.getTheta()),
					false);
		}

	}
	/**
	 * Converts the distance so that it can be used by the robot
	 * @param radius wheel radius
	 * @param distance distance the robot should move
	 * @return converted distance for the motors to use
	 */
	// Conversion method
	public boolean getIsTurning(){
		return this.isTurning;
	}
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	/**
	 * Converts angle so that it can be used by the robot to rotate
	 * @param radius wheel radius
	 * @param width distance between the two wheels
	 * @param angle angle the robot should turn 
	 * @return
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
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

	// helper method to read usData
	@Override
	public void processUSData(int distance) {

	}

	@Override
	public int readUSDistance() {
		return 0;
	}

	@Override
	public double readUSError() {
		// TODO Auto-generated method stub
		return 0;
	}

}