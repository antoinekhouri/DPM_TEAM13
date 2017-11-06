package ca.mcgill.ecse211.project13;

import lejos.hardware.Sound;

public class ObstacleAvoidance extends Thread{		
		
		
		private final int bandCenter = 10; 
		private Odometer odometer; 
		private UltrasonicPoller usPoller; 
		private Navigation navigation; 
		//added variables to record the next way point 
		private double posX; 
		private double posY; 
		
		public ObstacleAvoidance(Navigation navigation, Odometer odometer, UltrasonicPoller usPoller) {
			this.usPoller = usPoller; 
			this.navigation = navigation; 
			this.odometer = odometer; 
			
		}
		
		public void run() {
			while(true) {
				//when the robot is close to the block 
				
				if(usPoller.getDistance() < bandCenter) {
					//stop the robot and sound beep 
					MainProject.leftMotor.stop(); 
					MainProject.rightMotor.stop(); 
					Sound.beep(); 
					//record its destination 
					//posX = navigation.nextX; 
					//posY = navigation.nextY; 
					//enable avoid mode 
					avoid(); 
					//after going through the block, resume navigating to its destination 
					//navigation.travelTo(MainProject.usDistance, odometer, MainProject.WHEEL_RADIUS, MainProject.WHEEL_RADIUS, MainProject.TRACK, 4, 5); 
					}
			}
		}
		
		public void avoid() {
			//TO-DO
			//when we see a block head on, rotate the robot 90 degrees clockwise 
			//rotate the sensor 90 degrees anti-clockwise
			//p-controller or move forward until u dont see a brick any more
			//then bring back the sensor and brick to the original facing angle
			
			//turn 90 to avoid the block 
			//navigation.turnTo(90); 
			//travel 1/2 tile to bypass the block 
			//navigation.roll(1/2); 
			//turn -90 to check if bypassed the block 
			//navigation.turnTo(-90);
			//if not, call itself recursively until it went through the obstacle 
//			if(usPoller.getDistance() < bandCenter) {
//				avoid(); 
//			}
		}
		
		



}
