package ca.mcgill.ecse211.project13;

import ca.mcgill.ecse211.project13.Odometer;		
import lejos.hardware.motor.EV3LargeRegulatedMotor;
/**
 * This class implements the bridge crossing part(shallow water) of the project. In the MainProject class, 
 * after the robot is called to navigate to the start point of the bridge, the cross() method
 * of this class will be called to cross the bridge. The current implementation only supports
 * a corner shaped bridge, not a straight line bridge. The implementation first traverses the X
 * or Y portion of the bridge, depending on the bridge's shape, then turns 90 degrees in the appropriate 
 * direction and traverses the second portion of the bridge.
 * @author antoinekhouri
 *
 */

public class CrossBridge {
	private static final double TRACK = MainProject.TRACK;
	private static final double tileLength = 30.48;
	private static final int FORWARD_SPEED = 200;
	private static final int FORWARD_SPEED_RIGHT = (int) (FORWARD_SPEED*1.008);
	private static final int ROTATE_SPEED = 50;
	/**
	 * This method only supports a non straight-line bridge, i.e. a corner shaped bridge.
	 * @param endX the X coordinate of the end of the bridge
	 * @param endY the Y coordinate of the end of the bridge
	 * @param odometer the odometer used to poll the robot's current position.
	 * @param isGoingRight Boolean that indicates the direction of the bridge
	 * @param finalX End X coordinate of the bridge
	 * @param finalY End Y coordinate of the bridge
	 * 
	 */
	public void cross(double endX, double endY, Odometer odometer, boolean isGoingRight, int finalX, int finalY){

		endX = (1+endX)*tileLength;
		endY = (endY)*tileLength+tileLength/2;
		if(isGoingRight){

			MainProject.rightMotor.setSpeed(ROTATE_SPEED);
			MainProject.leftMotor.setSpeed(ROTATE_SPEED);
			//			MainProject.leftMotor.rotate(convertAngle(MainProject.WHEEL_RADIUS, TRACK, -90.0), true);
			//			MainProject.rightMotor.rotate(-convertAngle(MainProject.WHEEL_RADIUS, TRACK, -90.0), false);
			turnTo(0, odometer,MainProject.rightMotor,MainProject.leftMotor, MainProject.WHEEL_RADIUS,MainProject.WHEEL_RADIUS,
					TRACK);
			MainProject.rightMotor.setSpeed(FORWARD_SPEED_RIGHT);
			MainProject.leftMotor.setSpeed(FORWARD_SPEED);
			MainProject.leftMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, tileLength/2), true);
			MainProject.rightMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, tileLength/2), false);
			MainProject.rightMotor.setSpeed(ROTATE_SPEED);
			MainProject.leftMotor.setSpeed(ROTATE_SPEED);
			//			MainProject.leftMotor.rotate(convertAngle(MainProject.WHEEL_RADIUS, TRACK, 90.0), true);
			//			MainProject.rightMotor.rotate(-convertAngle(MainProject.WHEEL_RADIUS, TRACK, 90.0), false);
			turnTo(90, odometer,MainProject.rightMotor,MainProject.leftMotor, MainProject.WHEEL_RADIUS,MainProject.WHEEL_RADIUS,
					TRACK);
		}else{
			MainProject.rightMotor.setSpeed(ROTATE_SPEED);
			MainProject.leftMotor.setSpeed(ROTATE_SPEED);
			//			MainProject.leftMotor.rotate(convertAngle(MainProject.WHEEL_RADIUS, TRACK, 90.0), true);
			//			MainProject.rightMotor.rotate(-convertAngle(MainProject.WHEEL_RADIUS, TRACK, 90.0), false);
			turnTo(0, odometer,MainProject.rightMotor,MainProject.leftMotor, MainProject.WHEEL_RADIUS,MainProject.WHEEL_RADIUS,
					TRACK);
			MainProject.rightMotor.setSpeed(FORWARD_SPEED_RIGHT);
			MainProject.leftMotor.setSpeed(FORWARD_SPEED);
			MainProject.leftMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, tileLength/2), true);
			MainProject.rightMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, tileLength/2), false);
			MainProject.rightMotor.setSpeed(ROTATE_SPEED);
			MainProject.leftMotor.setSpeed(ROTATE_SPEED);
			//			MainProject.leftMotor.rotate(convertAngle(MainProject.WHEEL_RADIUS, TRACK, -90.0), true);
			//			MainProject.rightMotor.rotate(-convertAngle(MainProject.WHEEL_RADIUS, TRACK, -90.0), false);
			turnTo(270, odometer,MainProject.rightMotor,MainProject.leftMotor, MainProject.WHEEL_RADIUS,MainProject.WHEEL_RADIUS,
					TRACK);

		}
		MainProject.rightMotor.setSpeed(FORWARD_SPEED_RIGHT);
		MainProject.leftMotor.setSpeed(FORWARD_SPEED);
		MainProject.leftMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, endX-tileLength/2), true);
		MainProject.rightMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, endX-tileLength/2), false);
		if(isGoingRight){
			MainProject.rightMotor.setSpeed(ROTATE_SPEED);
			MainProject.leftMotor.setSpeed(ROTATE_SPEED);
			MainProject.leftMotor.rotate(convertAngle(MainProject.WHEEL_RADIUS, TRACK, 90.0), true);
			MainProject.rightMotor.rotate(-convertAngle(MainProject.WHEEL_RADIUS, TRACK, 90.0), false);
			//			turnTo(270, odometer,MainProject.rightMotor,MainProject.leftMotor, MainProject.WHEEL_RADIUS,MainProject.WHEEL_RADIUS,
			//					TRACK);
		}else{
			MainProject.rightMotor.setSpeed(ROTATE_SPEED);
			MainProject.leftMotor.setSpeed(ROTATE_SPEED);
			MainProject.leftMotor.rotate(convertAngle(MainProject.WHEEL_RADIUS, TRACK, -90.0), true);
			MainProject.rightMotor.rotate(-convertAngle(MainProject.WHEEL_RADIUS, TRACK, -90.0), false);
			//			turnTo(90, odometer,MainProject.rightMotor,MainProject.leftMotor, MainProject.WHEEL_RADIUS,MainProject.WHEEL_RADIUS,
			//					TRACK);
		}
		MainProject.rightMotor.setSpeed(FORWARD_SPEED_RIGHT);
		MainProject.leftMotor.setSpeed(FORWARD_SPEED);
		MainProject.leftMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, endY), true);
		MainProject.rightMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, endY), false);
		MainProject.rightMotor.setSpeed(ROTATE_SPEED);
		MainProject.leftMotor.setSpeed(ROTATE_SPEED);
		if(isGoingRight){				
			MainProject.leftMotor.rotate(convertAngle(MainProject.WHEEL_RADIUS, TRACK, -90.0), true);
			MainProject.rightMotor.rotate(-convertAngle(MainProject.WHEEL_RADIUS, TRACK, -90.0), false);
		}else{
			MainProject.leftMotor.rotate(convertAngle(MainProject.WHEEL_RADIUS, TRACK, 90.0), true);
			MainProject.rightMotor.rotate(-convertAngle(MainProject.WHEEL_RADIUS, TRACK, 90.0), false);	
		}
		MainProject.rightMotor.setSpeed(FORWARD_SPEED_RIGHT);
		MainProject.leftMotor.setSpeed(FORWARD_SPEED);
		MainProject.leftMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, tileLength/2), true);
		MainProject.rightMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, tileLength/2), false);
		odometer.setX((finalX+1)*tileLength);

	}
	/**
	 * This turning method is the same as the one used in navigation, as we
	 * realized it could simply be re-used here
	 * @param theta angle to turn to
	 * @param odometer	odometer used to poll robot's current position
	 * @param rightMotor right motor of the robot
	 * @param leftMotor left motor of the robot
	 * @param leftRadius robot's left wheel's radius
	 * @param rightRadius robot's right wheel's radius
	 * @param width distance between the two wheels
	 */
	private static void turnTo (double theta,Odometer odometer, EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor leftMotor,
			double leftRadius, double rightRadius, double width){


		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		while(theta-odometer.getTheta()>360){
			theta = theta-360;
		}
		while (theta-odometer.getTheta()<0){
			theta = theta+360;
		}
		if((theta-odometer.getTheta())>180){

			MainProject.rightMotor.rotate(convertAngle(MainProject.WHEEL_RADIUS, TRACK, 360-(theta-odometer.getTheta())), true);
			MainProject.leftMotor.rotate((-convertAngle(MainProject.WHEEL_RADIUS, TRACK, 360-(theta-odometer.getTheta()))), false);			  
		}
		else{
			MainProject.leftMotor.rotate((convertAngle(MainProject.WHEEL_RADIUS, TRACK, theta-odometer.getTheta())), true);
			MainProject.rightMotor.rotate(-convertAngle(MainProject.WHEEL_RADIUS, TRACK, theta-odometer.getTheta()), false);
		}


	}
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	/**
	 * Converts angle so that it can be used bendY the robot to rotate
	 * @param radius wheel radius
	 * @param TRACK distance between the two wheels
	 * @param angle angle the robot should turn 
	 * @return
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
