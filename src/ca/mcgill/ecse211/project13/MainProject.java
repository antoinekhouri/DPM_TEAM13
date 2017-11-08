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

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
/**
 * This is the main class that will run the robot. It will run all the threads that are required
 * for the robot to performed the desired actions.
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
		Done
	}
	//main
	private static State currState;
	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 9.1;
	private static final Port usPort = LocalEV3.get().getPort("S3");
	private static final Port lsPortLeft = LocalEV3.get().getPort("S2");
	private static final Port lsPortRight = LocalEV3.get().getPort("S4");
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
	private static final String SERVER_IP = "192.168.2.9";
	private static final int TEAM_NUMBER = 13;

	// Enable/disable printing of debug info from the WiFi class
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;
	private static int startPosition=4;
	public static int navX;
	public static int navY;
	public static int navBackX;
	public static int navBackY;
	public static int zipLineEndX;
	public static int zipLineEndY;
	public static int X0_final = 0;
	public static int Y0_final = 0;
	public static int XC_final = 0;
	public static int YC_final = 0;
	public static int bridgeX;
	public static int bridgeY;
	public static int friendlyZoneXStart;
	public static int friendlyZoneXEnd;
	public static int friendlyZoneYStart;
	public static int friendlyZoneYEnd;
	public static int enemyZoneXStart;
	public static int enemyZoneXEnd;
	public static int enemyZoneYStart;
	public static int enemyZoneYEnd;
	public static int bridgeEndX;
	public static int bridgeEndY;	
	public static double minDistance;
	private static final int bandCenter = 25; // Offset from the wall (cm)
	private static final int bandWidth = 3; // Width of dead band (cm)
	private static boolean isGreenTeam = false;
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
	public static void main(String[] args) throws InterruptedException {


		WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

		try {

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
				friendlyZoneYStart = ((Long) data.get("GreenLL__y")).intValue();
				friendlyZoneYEnd = ((Long) data.get("Green_UR_y")).intValue();
				enemyZoneXStart = ((Long) data.get("Red_LL_x")).intValue();
				enemyZoneXEnd = ((Long) data.get("Red_UR_x")).intValue();
				enemyZoneYStart = ((Long) data.get("Red_LL_y")).intValue();
				enemyZoneYEnd = ((Long) data.get("Red_UR_y")).intValue();
			}else{
				friendlyZoneXStart = ((Long) data.get("Red_LL_x")).intValue();
				friendlyZoneXEnd = ((Long) data.get("Red_UR_x")).intValue();
				friendlyZoneYStart = ((Long) data.get("Red_LL_y")).intValue();
				friendlyZoneYEnd = ((Long) data.get("Red_UR_y")).intValue();
				enemyZoneXStart = ((Long) data.get("Green_LL_x")).intValue();
				enemyZoneXEnd = ((Long) data.get("Green_UR_x")).intValue();
				enemyZoneYStart = ((Long) data.get("GreenLL__y")).intValue();
				enemyZoneYEnd = ((Long) data.get("Green_UR_y")).intValue();
			}
			if(isGreenTeam){
				startPosition = ((Long) data.get("GreenCorner")).intValue();
			} else{
				startPosition = ((Long) data.get("RedCorner")).intValue();
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
			zipLineEndX = ((Long) data.get("ZC_R_x")).intValue();
			zipLineEndY = ((Long) data.get("ZC_R_y")).intValue();
		} catch (Exception e) {
			System.err.println("Error: " + e.getMessage());
		}

		// Wait until user decides to end program
		Button.waitForAnyPress();
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

		int position = 4;

		do {
			t.clear();
			t.drawString("Press Enter ", 0, 0);
			t.drawString("to start", 0, 1);
			buttonChoice = Button.waitForAnyPress();

		} while (buttonChoice != Button.ID_ENTER);
		odometer.start();
		odometryDisplay.start();
		while(currState != State.Done && buttonChoice !=Button.ID_ESCAPE){
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
				buttonChoice = Button.waitForAnyPress();
				if (buttonChoice == Button.ID_ESCAPE) {
					System.exit(0);
				}
				else{
					setState(State.LightLocalizing);
				}
			}
			if(currState == State.LightLocalizing){
				LightLocalizer lightLocalizer = new LightLocalizer(odometer, colorValueLeft, colorDataLeft, colorValueRight,
						colorDataRight, startPosition);
				lightLocalizer.localize();
				setState(State.Navigating);
			}

			// create instance of navigation
			


			// set up navigation
			if(currState == State.Navigating){
				//TODO: implement proper logic for detecting when to go into avoidance
				final Navigation nav = new Navigation(colorValueLeft, colorDataLeft, colorValueRight, colorDataRight);

				nav.travelTo(usDistance, odometer, WHEEL_RADIUS, WHEEL_RADIUS, TRACK, navX,
						navY, usPoller);




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


				trav.traverse(XC_final, YC_final);
				LightLocalizer lightLocalizer = new LightLocalizer(odometer, colorValueLeft, colorDataLeft, colorValueRight,
						colorDataRight, zipLineEndX, zipLineEndY);
				lightLocalizer.localize();
				if(odometer.getX()>friendlyZoneXStart && odometer.getX()<friendlyZoneXEnd
						&& odometer.getY()>friendlyZoneYStart && odometer.getY()<friendlyZoneYEnd){
					setState(State.Done);
				}
				else if(odometer.getX()>enemyZoneXStart && odometer.getX()<enemyZoneXEnd
						&& odometer.getY()>enemyZoneYStart && odometer.getY()<enemyZoneYEnd){
					setState(State.FlagSearching);
				}
			}
			if(currState == State.CrossingBridge){
				cb.cross(bridgeEndX, bridgeEndY, odometer);
				if(odometer.getX()>friendlyZoneXStart && odometer.getX()<friendlyZoneXEnd
						&& odometer.getY()>friendlyZoneYStart && odometer.getY()<friendlyZoneYEnd){
					setState(State.Done);
				}
				else if(odometer.getX()>enemyZoneXStart && odometer.getX()<enemyZoneXEnd
						&& odometer.getY()>enemyZoneYStart && odometer.getY()<enemyZoneYEnd){
					setState(State.FlagSearching);
				}
			}
			if(currState == State.FlagSearching){
				//TODO: Implement search zone movement, US sensor sweeping and 
				//robot movement towards potential flag
				if(usPoller.getDistance()<200){
					//Move closer to Object
					setState(State.FlagSniffing);

				}
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
			if(currState == State.Avoiding){
				//TODO: implement avoidance
				@SuppressWarnings("unused")
				Thread avoid = new Thread() {
					/**
					 * Definition of the avoid method 
					 */
					public void run() {
						@SuppressWarnings("unused")
						PController pController = new PController(bandCenter, bandWidth);
						@SuppressWarnings("unused")
						float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are returned

						int option = 0;
						Printer.printMainMenu(); // Set up the display on the EV3 screen
						while (option == 0) // and wait for a button press. The button
							option = Button.waitForAnyPress(); // ID (option) determines what type of control to use
					}

				};
			}
		}
	}

}
