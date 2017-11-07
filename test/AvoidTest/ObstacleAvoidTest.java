package AvoidTest;


import lejos.hardware.Sound;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import NavigationTest.Navigation;
import NavigationTest.Odometer;
import ca.mcgill.ecse211.project13.MainProject;
		
public class ObstacleAvoidTest extends Thread{


			
			private final int bandCenter = 10; 
			private Odometer odometer; 
			private UltrasonicPoller usPoller; 
			private Navigation navigation;  
			private static final int FORWARD_SPEED = 185;
			private static final int FORWARD_SPEED_Right = 185;
			
			//is the following gonna work?
			@SuppressWarnings("resource")
			SensorModes usSensor = new EV3UltrasonicSensor(MainProject.usPort);
			final SampleProvider usDistance = usSensor.getMode("Distance");
			float[] usData = new float[usDistance.sampleSize()];

			
			public ObstacleAvoidTest(Navigation navigation, Odometer odometer, UltrasonicPoller usPoller) {
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
						avoid(); 
						//after going through the block, resume navigating to its destination 
						navigation.travelTo(usDistance, odometer, (double)MainProject.WHEEL_RADIUS, (double)MainProject.WHEEL_RADIUS, (double)MainProject.TRACK, (double)MainProject.X0_final,
								(double)MainProject.Y0_final); 
						}
				}
			}
			
			public void avoid() {
				//TO-DO
				
					//when we see a block head on, rotate the robot 90 degrees clockwise 
					navigation.turnTo(90.0, odometer, (double)MainProject.WHEEL_RADIUS, (double)MainProject.WHEEL_RADIUS, (double)MainProject.TRACK); 
					//rotate the sensor 90 degrees anti-clockwise
					
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


