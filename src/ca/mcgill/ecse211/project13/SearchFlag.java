package ca.mcgill.ecse211.project13;

import lejos.hardware.Sound;
//import lejos.hardware.motor.EV3LargeRegulatedMotor;
//import lejos.hardware.sensor.EV3ColorSensor;
//import lejos.hardware.sensor.SensorModes;
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
		
		public static double distance;
		private UltrasonicPoller usPoller; 
		private SampleProvider colorValueFront;
		private float[] colorDataRight;
		private SampleProvider colorValueRight;
		private float[] colorDataLeft;
		private SampleProvider colorValueLeft;
		private float[] colorDataFront;
		private boolean isDetectBlock;
		//private boolean isCorrectFlag = false;
		private static final int FORWARD_SPEED = 150;
		private static final int ROTATE_SPEED = 50;
		private static int color = 3;
		int x1,y1,x2,y2,x3,y3,x4,y4;
		double[] coordinates;
		double[] angles;
		
		public SearchFlag(Odometer odometer, SampleProvider colorValueFront, float[] colorDataFront, 
				UltrasonicPoller usPoller) 
		{
			this.colorValueFront = colorValueFront;
			this.colorDataFront = colorDataFront;
			this.usPoller = usPoller;
		
		}
		
		public void detect(double searchZoneX, double searchZoneY, double searchZoneX2, double searchZoneY2, Odometer odometer){
			
			MainProject.leftMotor.setSpeed(ROTATE_SPEED);
			MainProject.rightMotor.setSpeed(ROTATE_SPEED); 
			MainProject.leftMotor.rotate(convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, 360.0), true);
			MainProject.rightMotor.rotate(-convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, 360.0), true);
		 	
			double longestDistance = Math.sqrt(((searchZoneY2 - searchZoneY)*(searchZoneY2 - searchZoneY))+((searchZoneX2 - searchZoneX)*(searchZoneX2 - searchZoneX)));
			
		 	int i = 0;
		 	while (MainProject.leftMotor.isMoving() && MainProject.rightMotor.isMoving()) {
		 		
		 		if(usPoller.getDistance() < longestDistance) {
		 			isDetectBlock = true;
		 			coordinates[i] = usPoller.getDistance();
		 			angles[i] = odometer.getTheta();
		 			i++;
		 		}
		 	}
		 	
			final Navigation nav2 = new Navigation(colorValueLeft, colorDataLeft, colorValueRight, colorDataRight);
		 	for (int j = 0; j < coordinates.length-1; j++) {
		 		
		 		nav2.turnTo(angles[j], odometer, MainProject.WHEEL_RADIUS, MainProject.WHEEL_RADIUS, MainProject.TRACK);
		 		
		 		MainProject.leftMotor.setSpeed(FORWARD_SPEED);
		 		MainProject.rightMotor.setSpeed(FORWARD_SPEED); 
		 		MainProject.leftMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, coordinates[j]-3.0), true);
		 		MainProject.rightMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, coordinates[j]-3.0), false);
		 		if(getColorDataFront() == color){
		 			Sound.beep();
		 			Sound.beep();
		 			Sound.beep();

		 		}
		 	}
		 	
		}
		
		private float getColorDataFront() {
			colorValueFront.fetchSample(colorDataFront, 0);
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

	

