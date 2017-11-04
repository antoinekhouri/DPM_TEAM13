package ca.mcgill.ecse211.project13;

import ca.mcgill.ecse211.project13.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class CrossBridge {
	private static final double tileLength = 30.48;
	private static final int FORWARD_SPEED = 150;
	private static final int ROTATE_SPEED = 50;

	public void cross(int endX, int endY, Odometer odometer){
		
		if(endY*tileLength > odometer.getY())	{	
			turnTo(Math.toDegrees(Math.atan((endX*tileLength-odometer.getX())/(endY*tileLength-odometer.getY())))
					,odometer, MainProject.rightMotor, MainProject.leftMotor,MainProject.WHEEL_RADIUS,MainProject.WHEEL_RADIUS,MainProject.TRACK);
		}
		else if(endX*tileLength < odometer.getX()){	
			turnTo((-1)*Math.toDegrees(Math.atan((endY*tileLength-odometer.getY())/(endX*tileLength
					-odometer.getX()))) - 90,
					odometer, MainProject.rightMotor, MainProject.leftMotor,MainProject.WHEEL_RADIUS,MainProject.WHEEL_RADIUS,MainProject.TRACK);
		}
		else	{				
			turnTo((-1)*Math.toDegrees(Math.atan((endY*tileLength-odometer.getY())/(
					endX*tileLength-odometer.getX()))) + 90
					,odometer, MainProject.rightMotor, MainProject.leftMotor,MainProject.WHEEL_RADIUS,MainProject.WHEEL_RADIUS,MainProject.TRACK);
		}
		double distance = Math.sqrt((endX*tileLength-odometer.getX())*(endX*tileLength-odometer.getX())
	    		+(endY*tileLength-odometer.getY())*(endY*tileLength-odometer.getY()));
	    MainProject.leftMotor.setSpeed(FORWARD_SPEED);
	    MainProject.rightMotor.setSpeed(FORWARD_SPEED);

	    MainProject.leftMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, distance), true);
	    MainProject.rightMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, distance), false);
	}
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

			rightMotor.rotate(convertAngle(rightRadius, width, 360-(theta-odometer.getTheta())), true);
			leftMotor.rotate((-convertAngle(leftRadius, width, 360-(theta-odometer.getTheta()))), false);			  
		}
		else{
			leftMotor.rotate((convertAngle(leftRadius, width, theta-odometer.getTheta())), true);
			rightMotor.rotate(-convertAngle(rightRadius, width, theta-odometer.getTheta()), false);
		}

	}
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	/**
	 * Converts angle so that it can be used bendY the robot to rotate
	 * @param radius wheel radius
	 * @param MainProject.TRACK distance between the two wheels
	 * @param angle angle the robot should turn 
	 * @return
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
