package ca.mcgill.ecse211.project13;

//import ca.mcgill.ecse211.project13.UltrasonicLocalizer;
import ca.mcgill.ecse211.project13.UltrasonicPoller;
import ca.mcgill.ecse211.project13.UltrasonicController;	
import ca.mcgill.ecse211.project13.LightLocalizer;
import ca.mcgill.ecse211.project13.TraverseZipLine;
import ca.mcgill.ecse211.project13.UltrasonicLocalizer;
import ca.mcgill.ecse211.project13.Navigation;
//import ca.mcgill.ecse211.project13.LightLocalizer;
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
		Done
	}
	//main
	public static State currState;
	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 9.1;
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static final Port lsPortLeft = LocalEV3.get().getPort("S2");
	private static final Port lsPortRight = LocalEV3.get().getPort("S4");
	public static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3LargeRegulatedMotor pulleyMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
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
	/**
	 * Main method containing all the threads
	 * @param args not used
	 * @throws InterruptedException throws an interrupted exception if a thread is interrupted
	 */
	private static void setState(State s){
		currState = s;
	}
	@SuppressWarnings("unused")
	public static void main(String[] args) throws InterruptedException {

		setState(State.USLocalizing);

		int X0_var = 0;
		int Y0_var = 0;
		int Xc_var = 0;
		int Yc_var = 0;


		int buttonChoice;
		final TextLCD t = LocalEV3.get().getTextLCD();
		final Odometer odometer = new Odometer(leftMotor, rightMotor);
		OdometryCorrection odometryCorrection = new OdometryCorrection(odometer);
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

		//  LightLocalizer lightLocalizer = new LightLocalizer(odometer, colorValue, colorData);
		int position = 4;

		// have the user select the starting position of the robot using the buttons on the brick

		do {
			t.clear();
			t.drawString(" < Left   |   Right >", 0, 0);
			t.drawString("          |        ", 0, 1);
			t.drawString(" 0        |   1  ", 0, 2);
			t.drawString("          |        ", 0, 3);
			t.drawString("Up        |   Down    ", 0, 4);
			t.drawString("          |    ", 0, 5);
			t.drawString("2         |   3 ", 0, 6);
			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT
				&& buttonChoice != Button.ID_UP && buttonChoice != Button.ID_DOWN);
		if (buttonChoice == Button.ID_LEFT) {
			position = 0;
		} else if (buttonChoice == Button.ID_RIGHT) {
			position = 1;
		} else if (buttonChoice == Button.ID_UP) {
			position = 2;
		} else if (buttonChoice == Button.ID_DOWN) {
			position = 3;
		}


		// have the user select the coordinates of X0, Y0 by using the right button to increase X
		// and the up button to increase Y. They must click enter to move on to next screen
		do {
			t.clear();
			t.drawString("Enter (X0,Y0) ", 0, 0);
			t.drawString("          |    ", 0, 1);
			t.drawString("Press right to ", 0, 2);
			t.drawString(" increse X ", 0, 3);
			t.drawString("Press up to ", 0, 4);
			t.drawString("increse Y ", 0, 5);
			t.drawString("          |    ", 0, 6);
			t.drawString("press center to select ", 0, 7);
			buttonChoice = Button.waitForAnyPress();

			if (buttonChoice == Button.ID_RIGHT) {
				X0_var = X0_var + 1;
			} else if (buttonChoice == Button.ID_UP) {
				Y0_var = Y0_var + 1;
			} else if (buttonChoice == Button.ID_ENTER) {
				X0_final = X0_var;
				Y0_final = Y0_var;
			}
		} while (buttonChoice != Button.ID_ENTER);

		// have the user select the coordinates of XC, YX by using the right button to increase X
		// and the up button to increase Y. Pressing enter will start ultrasonic localization.
		do {
			t.clear();
			t.drawString("Enter (Xc,Yc) ", 0, 0);
			t.drawString("          |    ", 0, 1);
			t.drawString("Press right to ", 0, 2);
			t.drawString(" increse X ", 0, 3);
			t.drawString("Press up to ", 0, 4);
			t.drawString("increse Y ", 0, 5);
			t.drawString("          |    ", 0, 6);
			t.drawString("press center to select ", 0, 7);
			buttonChoice = Button.waitForAnyPress();

			if (buttonChoice == Button.ID_RIGHT) {
				Xc_var = Xc_var + 1;
			} else if (buttonChoice == Button.ID_UP) {
				Yc_var = Yc_var + 1;
			} else if (buttonChoice == Button.ID_ENTER) {
				XC_final = Xc_var;
				YC_final = Yc_var;
			}
		} while (buttonChoice != Button.ID_ENTER);
		while(currState != State.Done){
			UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(true, odometer);
			UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, usLocalizer);
			CrossBridge cb = new CrossBridge();
			final TraverseZipLine trav = new TraverseZipLine(odometer, colorValueLeft, colorDataLeft );
			odometer.start();
			odometryDisplay.start();
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
						colorDataRight);
				lightLocalizer.localize();
				setState(State.Navigating);
			}

			// start light localization
			//    lightLocalizer.localize(position);

			// create instance of navigation
			final Navigation navigation = new Navigation(true);

			// set up navigation
			if(currState == State.Navigating){
				//TODO: implement proper logic for detecting when to go into avoidance
				leftMotor.forward();
				leftMotor.flt();
				rightMotor.forward();
				rightMotor.flt();
				Thread move = new Thread() {
					/**
					 * Definition of the navigation thread
					 */
					public void run() {
						navigation.travelTo(usDistance, odometer, WHEEL_RADIUS, WHEEL_RADIUS, TRACK, X0_final,
								Y0_final, XC_final, YC_final);
					}

				};
				move.start();
				if(usPoller.getDistance()<minDistance){
					setState(State.Avoiding);
				}
				else if(Math.abs(odometer.getX()-XC_final)<10 && Math.abs(odometer.getY()-YC_final)<10){
					setState(State.Traversing);
				}else if(Math.abs(odometer.getX()-bridgeX)<10 && Math.abs(odometer.getY()-bridgeY)<10){
					setState(State.CrossingBridge);
				}
			}
			if(currState == State.Traversing){
				Thread traverse = new Thread(){
					public void run(){
						trav.traverse();
					}
				};
				traverse.start();
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
				setState(State.Navigating);
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
