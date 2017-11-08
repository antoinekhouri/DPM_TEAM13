package ca.mcgill.ecse211.project13;


import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;
/**
 * This class is what is used to make the robot attach to the zip line, traverse it and land
 * @author Veronica Nasseem, Nusaiba Radi, Antoine Khouri, Nikki Daly, Diana Serra, Asma Abdullah
 *
 */
public class TraverseZipLine {

	private static final int FORWARD_SPEED = 300;
	private static final int FORWARD_SLOW = 100;
	private static final int ROTATE_SPEED = 50;
	private static final double tileLength = 30.48;
	private Odometer odometer;
	private SampleProvider colorSensor;
	private float[] colorData;
	private static double lightDensity = 0.05;
	private double originalX;
	private double originalY;
	private double originalTheta;
	private boolean isXSet = false;
	private boolean isYSet = false;
	private boolean isOriginalThetaSet = false;
	/**
	 * Default constructor
	 * @param odometer Odometer used to get current position of robot
	 * @param colorSensor Sampleprovider for the light sensor
	 * @param colorData buffer for the light sensor's data
	 */
	public TraverseZipLine(Odometer odometer, SampleProvider colorSensor, float[] colorData){
		this.odometer = odometer;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
	}
	/**
	 * Method that gets the robot to properly traverse the zip line & land
	 */
	public void traverse(int xC, int yC){
		double theta;
		if(!isXSet){
			originalX = this.odometer.getX();
			isXSet = true;
		}
		if(!isYSet){
			originalY = this.odometer.getY();
			isYSet = true;
		}
		if (!isOriginalThetaSet){
			originalTheta = this.odometer.getTheta();
			isOriginalThetaSet = true;
		}

		if((Math.abs(xC*tileLength-odometer.getX()))<10 ){
	    	if(yC*tileLength-odometer.getY()>0){
	    		theta = 0;
	    		turnTo(theta,odometer,MainProject.WHEEL_RADIUS,MainProject.WHEEL_RADIUS,MainProject.TRACK);
	    		MainProject.leftMotor.stop(true);
	    		MainProject.rightMotor.stop(true);
	    	}
	    	else{
	    		theta = 180;
	    		turnTo(theta,odometer,MainProject.WHEEL_RADIUS,MainProject.WHEEL_RADIUS,MainProject.TRACK);
	    		MainProject.leftMotor.stop(true);
	    		MainProject.rightMotor.stop(true);
	    	}	
	    }
	    else if(Math.abs((yC*tileLength-odometer.getY()))<10){
	    	if(xC*tileLength-odometer.getX()>0){
	    		theta =90 ;
	    		turnTo(theta,odometer,MainProject.WHEEL_RADIUS,MainProject.WHEEL_RADIUS,MainProject.TRACK);
	    		MainProject.leftMotor.stop(true);
	    		MainProject.rightMotor.stop(true);
	    	}
	    	else{
	    		theta =270;
	    		turnTo(theta,odometer,MainProject.WHEEL_RADIUS,MainProject.WHEEL_RADIUS,MainProject.TRACK);
	    		MainProject.leftMotor.stop(true);
	    		MainProject.rightMotor.stop(true);
	    	}
	    	
	    }
	    else{
	    	
			 // if the x and y error is large it will change the angle of the robot
				if(yC*tileLength > odometer.getY())	{	
					turnTo(Math.toDegrees(Math.atan((xC*tileLength-odometer.getX())/(yC*tileLength-odometer.getY())))
							,odometer,MainProject.WHEEL_RADIUS,MainProject.WHEEL_RADIUS,MainProject.TRACK);
					MainProject.leftMotor.stop(true);
		    		MainProject.rightMotor.stop(true);
				}
				else if(xC*tileLength < odometer.getX()){	
					turnTo((-1)*Math.toDegrees(Math.atan((yC*tileLength-odometer.getY())/(xC*tileLength
							-odometer.getX()))) - 90,
					odometer,MainProject.WHEEL_RADIUS,MainProject.WHEEL_RADIUS,MainProject.TRACK);
					MainProject.leftMotor.stop(true);
		    		MainProject.rightMotor.stop(true);
				}
				else	{				
					turnTo((-1)*Math.toDegrees(Math.atan((yC*tileLength-odometer.getY())/(
							xC*tileLength-odometer.getX()))) + 90
							,odometer,MainProject.WHEEL_RADIUS,MainProject.WHEEL_RADIUS,MainProject.TRACK);
					MainProject.leftMotor.stop(true);
		    		MainProject.rightMotor.stop(true);
				}
			
	    }
		// move forward until you arrive on the zipline

		MainProject.rightMotor.setSpeed(FORWARD_SLOW);
		MainProject.leftMotor.setSpeed(FORWARD_SLOW);

		MainProject.leftMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, 20), true);
		MainProject.rightMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, 20), false);

		MainProject.pulleyMotor.setSpeed(FORWARD_SPEED);
		MainProject.pulleyMotor.forward();

		MainProject.rightMotor.setSpeed(FORWARD_SPEED);
		MainProject.leftMotor.setSpeed(FORWARD_SPEED);

		MainProject.leftMotor.forward();
		MainProject.rightMotor.forward();

		while (getColorData() < lightDensity) {
			//try-catch from ultrasonic poller
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				//	Auto-generated catch block

			}
		}
		Sound.beep();
		MainProject.rightMotor.stop(true);
		MainProject.leftMotor.stop(true);

		while (getColorData() > lightDensity){
			try{
				Thread.sleep(100);
			} catch (InterruptedException e){

			}
		}
		Sound.beep();
		MainProject.rightMotor.setSpeed(FORWARD_SPEED);
		MainProject.leftMotor.setSpeed(FORWARD_SPEED);
		MainProject.pulleyMotor.setSpeed(FORWARD_SLOW);

		MainProject.rightMotor.forward();
		MainProject.leftMotor.forward();
		MainProject.pulleyMotor.forward();
		while(getColorData() <0.3){

		} try{
			Thread.sleep(100);
		} catch (InterruptedException e){

		}
		Sound.beep();
		MainProject.rightMotor.setSpeed(FORWARD_SLOW);
		MainProject.leftMotor.setSpeed(FORWARD_SLOW);
		MainProject.pulleyMotor.stop(true);
		if(Math.abs(originalTheta-0)<10){
			odometer.setY(originalY+5*tileLength);
		}else if(Math.abs(originalTheta-90)<10){
			odometer.setX(originalX+5*tileLength);
		}
		else if(Math.abs(originalTheta-180)<10){
			odometer.setY(originalY-5*tileLength);
		}else{
			odometer.setX(originalX-5*tileLength);
		}
		MainProject.rightMotor.stop(true);
		MainProject.leftMotor.stop(true);
	}
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
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	/**
	 * Processes the data of the light sensor
	 * @return color brightness level 
	 */
	private float getColorData() {
		colorSensor.fetchSample(colorData, 0);
		float colorBrightnessLevel = (colorData[0] + colorData[1] + colorData[2]);
		return colorBrightnessLevel;
	}
}
