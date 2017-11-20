package ca.mcgill.ecse211.project13;


import ca.mcgill.ecse211.project13.UltrasonicPoller;		
import ca.mcgill.ecse211.project13.LightLocalizer;
import ca.mcgill.ecse211.project13.TraverseZipLine;
import ca.mcgill.ecse211.project13.UltrasonicLocalizer;
import ca.mcgill.ecse211.project13.Navigation;
import ca.mcgill.ecse211.project13.Odometer;
import ca.mcgill.ecse211.project13.OdometryDisplay;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

import java.io.IOException;
import java.net.UnknownHostException;
import java.util.Map;

import org.json.simple.parser.ParseException;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
/**
 * This is the main class that will run the robot. It will run all the threads and call all
 * the methods that are required
 * for the robot to performed the desired actions.
 * This class implements a Finite State Machine in order to determine what actions the robot
 * should take and the sequence of actions required
 * @author Veronica Nasseem, Nusaiba Radi, Antoine Khouri, Nikki Daly, Diana Serra, Asma Abdullah
 *
 */
public class MainProject {

	enum State{
		Navigating,
		Avoiding,
		USLocalizing,
		LightLocalizing,
		Traversing,
		FlagSearching,
		FlagSniffing,
		CrossingBridge,
		FlagFound,
		NavigatingBack,
		GoHome,
		Done
	}

	private static State currState;
	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 9.40;
	private static final Port usPort = LocalEV3.get().getPort("S3");
	private static final Port lsPortLeft = LocalEV3.get().getPort("S1");
	private static final Port lsPortRight = LocalEV3.get().getPort("S2");
	private static final Port lsFront = LocalEV3.get().getPort("S4");
	public static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3LargeRegulatedMotor pulleyMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	// the motor to rotate the sensor
	public static final EV3LargeRegulatedMotor sMotor = 
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));


	// ** Set these as appropriate for your team and current situation **
	private static final String SERVER_IP = "192.168.2.27";


	private static final int TEAM_NUMBER = 13;

	// Enable/disable printing of debug info from the WiFi class
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

	public static double minDistance;
	private static final int bandCenter = 25; // Offset from the wall (cm)
	private static final int bandWidth = 3; // Width of dead band (cm)
	private static boolean isGreenTeam = false;
	private static final double tileLength = 30.48;
	/**
	 * Main method containing all the threads
	 * @param args not used
	 * @throws InterruptedException throws an interrupted exception if a thread is interrupted
	 */
	public static void setState(State s){
		currState = s;
	}
	public static  String getState(){
		return currState.toString();
	}
	@SuppressWarnings("unused")
	/**
	 * Main method that uses wifi data as input as well as implements the state based logic
	 * for this project.
	 * @param args unused 
	 * @throws InterruptedException 
	 * @throws UnknownHostException 
	 * @throws IOException
	 * @throws ParseException
	 */
	public static void main(String[] args) throws InterruptedException, UnknownHostException, IOException, ParseException {

		boolean isZipLineDiagonal = false;
		int startPosition=0;
		double finalX=0;
		double finalY=0;
		double finalTheta=0;
		int navX=0;
		int navY=0;
		int navBackX=0;
		int navBackY=0;
		int zipLineEndX=0;
		int zipLineEndY=0;
		int X0_final = 0;
		int Y0_final = 0;
		int XC_final = 0;
		int YC_final = 0;
		int bridgeX=0;
		int bridgeY=0;
		int friendlyZoneXStart=0;
		int friendlyZoneXEnd=0;
		int friendlyZoneYStart=0;
		int friendlyZoneYEnd=0;
		int enemyZoneXStart=0;
		int enemyZoneXEnd=0;
		int enemyZoneYStart=0;
		int enemyZoneYEnd=0;
		int bridgeEndX=0;
		int bridgeEndY=0;
		int zipLineRedX=0;
		int zipLineRedY=0;
		//lower left
		int searchZoneX=0;
		int searchZoneY=0;
		//upperright
		int searchZoneX2=0;
		int searchZoneY2=0;
		int bridgeUpperRightX=0;
		int bridgeUpperRightY=0;
		boolean isZipLineVertical = false;
		WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, !ENABLE_DEBUG_WIFI_PRINT);



		Map data = conn.getData();

		// Example 1: Print out all received data
		//			System.out.println("Map:\n" + data);

		// Example 2 : Print out specific values

		int redTeam = ((Long) data.get("RedTeam")).intValue();
		if(redTeam == 13){
			isGreenTeam = false;
		}
		int greenTeam = ((Long) data.get("GreenTeam")).intValue();
		if(greenTeam == 13){
			isGreenTeam = true;
		}
		if(isGreenTeam){
			friendlyZoneXStart = ((Long) data.get("Green_LL_x")).intValue();
			friendlyZoneXEnd = ((Long) data.get("Green_UR_x")).intValue();
			friendlyZoneYStart = ((Long) data.get("Green_LL_y")).intValue();
			friendlyZoneYEnd = ((Long) data.get("Green_UR_y")).intValue();
			enemyZoneXStart = ((Long) data.get("Red_LL_x")).intValue();
			enemyZoneXEnd = ((Long) data.get("Red_UR_x")).intValue();
			enemyZoneYStart = ((Long) data.get("Red_LL_y")).intValue();
			enemyZoneYEnd = ((Long) data.get("Red_UR_y")).intValue();
			searchZoneX = ((Long) data.get("SR_LL_x")).intValue();
			searchZoneY = ((Long) data.get("SR_LL_y")).intValue();
		}else{
			friendlyZoneXStart = ((Long) data.get("Red_LL_x")).intValue();
			friendlyZoneXEnd = ((Long) data.get("Red_UR_x")).intValue();
			friendlyZoneYStart = ((Long) data.get("Red_LL_y")).intValue();
			friendlyZoneYEnd = ((Long) data.get("Red_UR_y")).intValue();
			enemyZoneXStart = ((Long) data.get("Green_LL_x")).intValue();
			enemyZoneXEnd = ((Long) data.get("Green_UR_x")).intValue();
			enemyZoneYStart = ((Long) data.get("Green_LL__y")).intValue();
			enemyZoneYEnd = ((Long) data.get("Green_UR_y")).intValue();
			searchZoneX = ((Long) data.get("SG_LL_x")).intValue();
			searchZoneY = ((Long) data.get("SG_LL_y")).intValue();
		}
		if(isGreenTeam){
			startPosition = ((Long) data.get("GreenCorner")).intValue();
		} else{
			startPosition = ((Long) data.get("RedCorner")).intValue();
		}
		if(startPosition == 0){
			finalX = 1;
			finalY = 1;
			finalTheta = 0;
		}else if(startPosition ==1){
			finalX =7;
			finalY =1;
			finalTheta = 0;
		}else if(startPosition ==2){
			finalX = 7;
			finalY = 7;
			finalTheta = 180;
		}else{
			finalX = 1;
			finalY = 7;
			finalTheta = 180;
		}
		if(isGreenTeam){
			navX = ((Long) data.get("ZO_G_x")).intValue();
			navY = ((Long) data.get("ZO_G_y")).intValue();
			navBackX = ((Long) data.get("SH_LL_x")).intValue();
			navBackY = ((Long) data.get("SH_LL_y")).intValue();
		}else{
			navX = ((Long) data.get("SH_LL_x")).intValue();
			navY = ((Long) data.get("SH_LL_y")).intValue();
			navBackX = ((Long) data.get("ZO_G_x")).intValue();
			navBackY = ((Long) data.get("ZO_G_y")).intValue();
		}
		XC_final = ((Long) data.get("ZC_G_x")).intValue();
		YC_final = ((Long) data.get("ZC_G_y")).intValue();
		zipLineRedX = ((Long) data.get("ZC_R_x")).intValue();
		zipLineRedY = ((Long) data.get("ZC_R_y")).intValue();
		zipLineEndX = ((Long) data.get("ZO_R_x")).intValue();
		zipLineEndY = ((Long) data.get("ZO_R_y")).intValue();
		bridgeUpperRightX = ((Long) data.get("SH_UR_x")).intValue();
		bridgeUpperRightY = ((Long) data.get("SH_UR_y")).intValue();
		bridgeEndX = ((Long) data.get("SV_LL_x")).intValue();
		bridgeEndY = ((Long) data.get("SV_LL_y")).intValue();
		if(XC_final==zipLineRedX){
			isZipLineVertical=true;

		}else if (YC_final==zipLineRedY){
			isZipLineVertical=false;
		}else if(YC_final != zipLineRedY && XC_final!=zipLineRedX){
			isZipLineDiagonal=true;
		}
		

		// Wait until user decides to end program
//		Button.waitForAnyPress();
		setState(State.USLocalizing);

		int X0_var = 0;
		int Y0_var = 0;
		int Xc_var = 0;
		int Yc_var = 0;


		int buttonChoice;
		final TextLCD t = LocalEV3.get().getTextLCD();
		final Odometer odometer = new Odometer(leftMotor, rightMotor);
		//		OdometryCorrection odometryCorrection = new OdometryCorrection(odometer);
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t);

		@SuppressWarnings("resource")
		SensorModes usSensor = new EV3UltrasonicSensor(usPort);
		final SampleProvider usDistance = usSensor.getMode("Distance");
		float[] usData = new float[usDistance.sampleSize()];

		@SuppressWarnings("resource")
		SensorModes colorSensorLeft = new EV3ColorSensor(lsPortLeft);
		SampleProvider colorValueLeft = colorSensorLeft.getMode("Red");
		float[] colorDataLeft = new float[3];

		@SuppressWarnings("resource")
		SensorModes colorSensorRight = new EV3ColorSensor(lsPortRight);
		SampleProvider colorValueRight = colorSensorRight.getMode("Red");
		float[] colorDataRight = new float[3];
		
		@SuppressWarnings("resource")
		SensorModes colorSensorFront = new EV3ColorSensor(lsFront);
		SampleProvider colorValueFront = colorSensorRight.getMode("Red");
		float[] colorDataFront = new float[3];

		int position = 4;

//		do {
//			t.clear();
//			t.drawString("Press Enter ", 0, 0);
//			t.drawString("to start", 0, 1);
//			buttonChoice = Button.waitForAnyPress();
//		}
//		} while (buttonChoice != Button.ID_ENTER);
		odometer.start();
		odometryDisplay.start();
		while(currState != State.Done){
			UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(true, odometer);
			final UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, usLocalizer);
			CrossBridge cb = new CrossBridge();
			final TraverseZipLine trav = new TraverseZipLine(odometer, colorValueLeft, colorDataLeft );

			// start the odometer and the display
			if(currState == State.USLocalizing){


				usPoller.start();

				// start the ultrasonic localization. Once it is done, it will wait for a button to be pressed
				// to proceed
				// if escape is pressed, the program will stop
								//buttonChoice = Button.waitForAnyPress();
//								if (buttonChoice == Button.ID_ESCAPE) {
//									System.exit(0);
//								}
//								else{
				while(!usLocalizer.getIsDone()){
					Thread.sleep(1);
				}
				setState(State.LightLocalizing);
//				}
				//				}
			}
			if(currState == State.LightLocalizing){
				LightLocalizer lightLocalizer = new LightLocalizer(odometer, colorValueLeft, colorDataLeft, colorValueRight,
						colorDataRight, finalX, finalY, finalTheta);
				lightLocalizer.localize(finalX, finalY, finalTheta, false); // corner 1 : 1,1,0;
				//corner 2: 7,1,0
				//corner 3: 7,7,180
				//corner 4: 1,7,180
//								buttonChoice = Button.waitForAnyPress();
//								if(buttonChoice == Button.ID_ESCAPE){
//									setState(State.Done);
//								}
//								else{
				while(!lightLocalizer.getIsDone()){
					Thread.sleep(1);
				}
				setState(State.Navigating);
//								}
			}

			// create instance of navigation



			// set up navigation
			if(currState == State.Navigating){
				//TODO: implement proper logic for detecting when to go into avoidance
				final Navigation nav = new Navigation(colorValueLeft, colorDataLeft, colorValueRight, colorDataRight);
				if(!isGreenTeam){
					if(navX>finalX){
						navX= navX-1;
					}else{
						navX = navX+1;
					}
				}
				if(isZipLineVertical){
					nav.travelTo(usDistance, odometer, WHEEL_RADIUS, WHEEL_RADIUS, TRACK,navX,
							navY,finalX,finalY, usPoller, isZipLineVertical);//1,1 = depending on starting corner
				}else{
					nav.travelTo(usDistance, odometer, WHEEL_RADIUS, WHEEL_RADIUS, TRACK,navX,
							navY,finalX,finalY, usPoller);
				}



				//				buttonChoice = Button.waitForAnyPress();
				//				if(buttonChoice == Button.ID_ESCAPE){
				//					setState(State.Done);
				//				}
				if(usPoller.getDistance()<minDistance){
					setState(State.Avoiding);
				}
				else if(isGreenTeam){
					setState(State.Traversing);
				}else{
					setState(State.CrossingBridge);
				}					
			}
			if(currState == State.Traversing){


				trav.traverse(XC_final,YC_final);
				LightLocalizer lightLocalizer = new LightLocalizer(odometer, colorValueLeft, colorDataLeft, colorValueRight,
						colorDataRight);
//				buttonChoice = Button.waitForAnyPress();
				//				if(buttonChoice == Button.ID_ESCAPE){
				//					setState(State.Done);
				//					break;
				//				}
				if(isGreenTeam){
					if(isZipLineDiagonal){
						if(zipLineRedX>XC_final && zipLineRedY>YC_final){
							finalTheta = 90;
						}else if(zipLineRedX>XC_final && zipLineRedY<YC_final){
							finalTheta = 180;
						}else if(zipLineRedX<XC_final && zipLineRedY>YC_final){
							finalTheta = 270;
						}else{
							finalTheta = 0;
						}
					}
					else if(zipLineRedX>XC_final){
						finalTheta=180;
					}else if(zipLineRedX<XC_final){
						finalTheta =0;
					}else if(zipLineRedY>YC_final){
						finalTheta=90;
					}else{
						finalTheta=270;
					}
				}
				lightLocalizer.localize(zipLineEndX, zipLineEndY,finalTheta,true);
				//				buttonChoice = Button.waitForAnyPress();
				//				if(buttonChoice == Button.ID_ESCAPE){
				//					setState(State.Done);
				//				}
				if(!isGreenTeam){
					setState(State.GoHome);
				}
				else if(isGreenTeam){
					setState(State.FlagSearching);
				}

			}
			if(currState == State.CrossingBridge){
				int bridgeTravelX=0;
				int bridgeTravelY=0;
				if(!isGreenTeam){
					bridgeTravelX = bridgeUpperRightX-navX+1;
					bridgeTravelY = bridgeEndY-bridgeUpperRightY;
				}else{
					bridgeTravelX = bridgeUpperRightX-navBackX+1;
					bridgeTravelY = bridgeEndY-bridgeUpperRightY;
				}
				cb.cross(bridgeTravelX, bridgeTravelY, odometer);
				if(isGreenTeam){
					setState(State.GoHome);
				}
				else {
					setState(State.FlagSearching);
				}
			}
			if(currState == State.FlagSearching){
				final Navigation nav2 = new Navigation(colorValueLeft, colorDataLeft, colorValueRight, colorDataRight);
				if(!isZipLineVertical){
					nav2.travelTo(usDistance, odometer, WHEEL_RADIUS, WHEEL_RADIUS, TRACK,searchZoneX, searchZoneY, zipLineEndX, zipLineEndY, usPoller, isZipLineVertical);
				}else{
					nav2.travelTo(usDistance, odometer, WHEEL_RADIUS, WHEEL_RADIUS, TRACK,searchZoneX, searchZoneY, zipLineEndX, zipLineEndY, usPoller);
				}
				setState(State.NavigatingBack);
				final SearchFlag search = new SearchFlag();
				search.detect(searchZoneX, searchZoneY, searchZoneX2, searchZoneY2, odometer);
				
				setState(State.Done);
				break;


				
			}
			if(currState == State.FlagSniffing){
				//TODO: implement light sensor sniffing code
				//if correct flag->
				if(true){
					setState(State.FlagFound);
				}
				else{
					setState(State.FlagSearching);
				}
			}
			if(currState == State.FlagFound){
				Sound.beep();
				Sound.beep();
				Sound.beep();
				setState(State.NavigatingBack);
			}
			if(currState == State.NavigatingBack){
				final Navigation nav3 = new Navigation(colorValueLeft, colorDataLeft, colorValueRight, colorDataRight);
				if(!isZipLineVertical){
					nav3.travelTo(usDistance, odometer, WHEEL_RADIUS, WHEEL_RADIUS, TRACK,navBackX, navBackY, searchZoneX, searchZoneY, usPoller, isZipLineVertical);
				}else{
					nav3.travelTo(usDistance, odometer, WHEEL_RADIUS, WHEEL_RADIUS, TRACK,navBackX, navBackY, searchZoneX, searchZoneY, usPoller);
				}
				if (isGreenTeam){
					setState(State.CrossingBridge);
				}else{
					setState(State.Traversing);
				}
			}
			if(currState == State.GoHome){
				final Navigation nav4 = new Navigation(colorValueLeft, colorDataLeft, colorValueRight, colorDataRight);
				if(!isZipLineVertical){
					nav4.travelTo(usDistance, odometer, WHEEL_RADIUS, WHEEL_RADIUS, TRACK,finalX, finalY, (int) odometer.getX()/tileLength, (int) odometer.getY()/tileLength, usPoller, isZipLineVertical);
				}else{
					nav4.travelTo(usDistance, odometer, WHEEL_RADIUS, WHEEL_RADIUS, TRACK,finalX, finalY, (int) odometer.getX()/tileLength, (int) odometer.getY()/tileLength, usPoller);
				}
				setState(State.Done);
				break;
			}
			if(currState == State.Avoiding){
				//TODO: implement avoidance
				@SuppressWarnings("unused")
				Thread avoid = new Thread() {
					/**
					 * Definition of the avoid method 
					 */
					public void run() {
//						@SuppressWarnings("unused")
//						PController pController = new PController(bandCenter, bandWidth);
						@SuppressWarnings("unused")
						float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are returned

						int option = 0;
//						Printer.printMainMenu(); // Set up the display on the EV3 screen
						while (option == 0) // and wait for a button press. The button
							option = Button.waitForAnyPress(); // ID (option) determines what type of control to use
					}

				};
			}
		}
	
		if(currState==State.Done){
			leftMotor.stop(true);
			rightMotor.stop(true);
			System.exit(0);
		}
	} 

}
