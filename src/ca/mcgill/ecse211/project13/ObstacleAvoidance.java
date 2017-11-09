package ca.mcgill.ecse211.project13;

import lejos.hardware.Sound;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.sensor.EV3UltrasonicSensor;


public class ObstacleAvoidance extends Thread{		
		
		
		private final int bandCenter = 10; 
		private Odometer odometer; 
		private UltrasonicPoller usPoller; 
		private Navigation navigation;  
		private static final int FORWARD_SPEED = 185;
		private static final int FORWARD_SPEED_Right = 185;
		private SampleProvider usDistance;
		
		public ObstacleAvoidance(Navigation navigation, Odometer odometer, UltrasonicPoller usPoller, SampleProvider usDistance) {
			this.usPoller = usPoller; 
			this.navigation = navigation; 
			this.odometer = odometer; 
			this.usDistance = usDistance;

		}
		
		public void run() {
			while(true) {
				//when the robot is close to the block 
				
				if(usPoller.getDistance() < bandCenter) {
					//stop the robot and sound beep 
					MainProject.leftMotor.stop(); 
					MainProject.rightMotor.stop(); 
					Sound.beep(); 
					avoid(); 
					//after going through the block, resume navigating to its destination 
//					navigation.travelTo(usDistance, odometer, MainProject.WHEEL_RADIUS, MainProject.WHEEL_RADIUS, MainProject.TRACK, MainProject.X0_final,
//							MainProject.Y0_final,1,1, usPoller); 
					}
			}
		}
		
		public void avoid() {
			
				//when we see a block head on, rotate the robot 90 degrees clockwise 
				navigation.turnTo(90, odometer, MainProject.WHEEL_RADIUS, MainProject.WHEEL_RADIUS, MainProject.TRACK); 
				//rotate the sensor 90 degrees anti-clockwise
				MainProject.sMotor.rotate(-90);
				//p-controller or move forward until u don't see a brick any more
				while(usPoller.getDistance() < bandCenter) {
					MainProject.leftMotor.setSpeed(FORWARD_SPEED);
				    MainProject.rightMotor.setSpeed(FORWARD_SPEED_Right);		
				    MainProject.leftMotor.forward();
				    MainProject.rightMotor.forward();
				}
				//then bring back the sensor and brick to the original facing angle
				navigation.turnTo(-90, odometer, MainProject.WHEEL_RADIUS, MainProject.WHEEL_RADIUS, MainProject.TRACK); 
				MainProject.sMotor.rotate(90);
				
		}
		
		



}
