package ca.mcgill.ecse211.project13;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class SearchFlag {

	/*after robot navigates to the lower left point of the search zone
	 * it sweeps the area with the US sensor
	 * it records at what distances each block is
	 * can it calculate the coordinates?
	 * it moves to the upper right corner and sweeps
	 * if it can calculate coordinates then it records any new blocks it sees
	 * it goes and sniffs every block until it find the one it's looking for (beeps)
	 * if it can calculate coordinates, it doesn't need to go back to the original point after each sniff
	*/
	
	//Variables
	
	//something to store usDistances of each block (do we know how many blocks we're going to have?)
	public static double distance;
	private Odometer odometer;
	private UltrasonicPoller usPoller; 
	private SampleProvider colorSensorFront;
	private float[] colorDataFront;
	private boolean isDetectBlock = false;
	private boolean isCorrectFlag = false;
	private static final int FORWARD_SPEED = 150;
	private static final int ROTATE_SPEED = 50;
	private static double lightDensity = 0.40;

	int x1,y1,x2,y2,x3,y3,x4,y4;
	int[] coordinates;
	
	public void detect(int xLL, int yLL, int xUR, int yUR, Odometer odometer){
		
		MainProject.leftMotor.setSpeed(ROTATE_SPEED);
	    MainProject.rightMotor.setSpeed(ROTATE_SPEED); 
	 	MainProject.leftMotor.rotate(convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, 360.0), true);
	 	MainProject.rightMotor.rotate(-convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, 360.0), true);
	 	
		double longestDistance = Math.sqrt(((yUR - yLL)*(yUR - yLL))+((xUR - xLL)*(xUR - xLL)));
		
	 	int i = 0;
	 	while (MainProject.leftMotor.isMoving() && MainProject.rightMotor.isMoving()) {
	 		
	 		if(usPoller.getDistance() < longestDistance) {
	 			isDetectBlock = true;
	 			coordinates[i] = usPoller.getDistance();
	 			i++;
	 		}
	 	}
	 	for (int j = 0; j < coordinates.length-1; j++) {
	 		MainProject.leftMotor.setSpeed(FORWARD_SPEED);
	 		MainProject.rightMotor.setSpeed(FORWARD_SPEED); 
	 		MainProject.leftMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, coordinates[j]), true);
	 		MainProject.rightMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, coordinates[j]), false);
	 		if(getColorDataFront() < lightDensity+0.1 && getColorDataFront() > lightDensity-0.1){
	 			Sound.beep();
	 		}
	 	}
	 	
	}
	
	private float getColorDataFront() {
		colorSensorFront.fetchSample(colorDataFront, 0);
		float colorBrightnessLevel = (colorDataFront[0] + colorDataFront[1] + colorDataFront[2]);
		return colorBrightnessLevel;
	}
	
		private static int convertDistance(double radius, double distance) {
		   return (int) ((180.0 * distance) / (Math.PI * radius));
		}
		
	    private static int convertAngle(double radius, double width, double angle) {
	    	return convertDistance(radius, Math.PI * width * angle / 360.0);
	    }
	    
}
