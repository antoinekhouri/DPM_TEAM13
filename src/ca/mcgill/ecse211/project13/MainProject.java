package ca.mcgill.ecse211.project13;

//import ca.mcgill.ecse211.project13.UltrasonicLocalizer;
import ca.mcgill.ecse211.project13.UltrasonicPoller;
import ca.mcgill.ecse211.project13.Navigation;
//import ca.mcgill.ecse211.project13.LightLocalizer;
import ca.mcgill.ecse211.project13.Odometer;
import ca.mcgill.ecse211.project13.OdometryDisplay;
import lejos.hardware.Button;
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

  public static final double WHEEL_RADIUS = 2.1;
  public static final double TRACK = 9.7;
  private static final Port usPort = LocalEV3.get().getPort("S1");
  private static final Port lsPort = LocalEV3.get().getPort("S2");
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
  private static final int bandCenter = 25; // Offset from the wall (cm)
  private static final int bandWidth = 3; // Width of dead band (cm)
  /**
   * Main method containing all the threads
   * @param args not used
   * @throws InterruptedException throws an interrupted exception if a thread is interrupted
   */
  public static void main(String[] args) throws InterruptedException {


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
    SensorModes colorSensor = new EV3ColorSensor(lsPort);
    SampleProvider colorValue = colorSensor.getMode("Red");
    float[] colorData = new float[3];

//    LightLocalizer lightLocalizer = new LightLocalizer(odometer, colorValue, colorData);
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

    // create instances of ultrasonic localizer, poller and traverse zipline classes

//    UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(true, odometer, position);
//    UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, usLocalizer);
//    final TraverseZipLine trav = new TraverseZipLine(odometer);

    // start the odometer and the display
    odometer.start();
    odometryDisplay.start();

    // start the ultrasonic localization. Once it is done, it will wait for a button to be pressed
    // to proceed
    // if escape is pressed, the program will stop
//    usPoller.start();
    buttonChoice = Button.waitForAnyPress();
    if (buttonChoice == Button.ID_ESCAPE) {
      System.exit(0);
    }

    // start light localization
//    lightLocalizer.localize(position);

    // create instance of navigation
    final Navigation navigation = new Navigation(true);

    // set up navigation

    leftMotor.forward();
    leftMotor.flt();
    rightMotor.forward();
    rightMotor.flt();

    // create thread for navigation
    Thread move = new Thread() {
      /**
       * Definition of the navigation thread
       */
      public void run() {
        navigation.travelTo(usDistance, odometer, WHEEL_RADIUS, WHEEL_RADIUS, TRACK, X0_final,
            Y0_final, XC_final, YC_final);
      }

    };
    
    
    
    Thread avoid = new Thread() {
    	/**
    	 * Definition of the avoid method 
    	 */
    	public void run() {
    			PController pController = new PController(bandCenter, bandWidth);
    		    float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are returned

    			int option = 0;
    		    Printer.printMainMenu(); // Set up the display on the EV3 screen
    		    while (option == 0) // and wait for a button press. The button
    		      option = Button.waitForAnyPress(); // ID (option) determines what type of control to use

    			// Setup Printer
    		    // This thread prints status information in the background
    		    Printer printer = null;

    		    // Setup Ultrasonic Poller // This thread samples the US and invokes
    		    UltrasonicPoller usPoller = null; // the selected controller on each cycle

            usPoller = new UltrasonicPoller(usDistance, usData, pController);
            printer = new Printer(option, pController);
            usPoller.start();
            printer.start();
          }
    	
    };

    // create thread for traversing the zipline
//    Thread traverse = new Thread() {
//      public void run() {
//        trav.traverse();
//      }
//    };

    // if escape is pressed, the program will stop
    buttonChoice = Button.waitForAnyPress();
    if (buttonChoice == Button.ID_ESCAPE) {
      System.exit(0);
    }

    // start navigation
    move.start();

    // escape program is escape button is pressed
    buttonChoice = Button.waitForAnyPress();
    if (buttonChoice == Button.ID_ESCAPE) {
      System.exit(0);
    }

    // start traversing zipline once the robot reaches the right coordinates
//    traverse.start();
//    traverse.join();

    // escape program when escape button is pressed
    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
    


    int option = 0;
    Printer.printMainMenu(); // Set up the display on the EV3 screen
    while (option == 0) // and wait for a button press. The button
      option = Button.waitForAnyPress(); // ID (option) determines what type of control to use

    // Setup controller objects

//    BangBangController bangbangController =
//        new BangBangController(bandCenter, bandWidth, motorLow, motorHigh);

    PController pController = new PController(bandCenter, bandWidth);

    // Setup ultrasonic sensor
    // There are 4 steps involved:
    // 1. Create a port object attached to a physical port (done already above)
    // 2. Create a sensor instance and attach to port
    // 3. Create a sample provider instance for the above and initialize operating mode
    // 4. Create a buffer for the sensor data

    @SuppressWarnings("resource") // Because we don't bother to close this resource
    //SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
    //@SuppressWarnings("resource")
	//SensorModes tSensor = new EV3UltrasonicSensor(tPort);
    
    //SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from this instance
    //SampleProvider tDistance = tSensor.getMode(0);
    
    //float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are returned
    //float[] tData = new float[tDistance.sampleSize()];                                                    

    // Setup Printer
    // This thread prints status information in the background
    Printer printer = null;

    // Setup Ultrasonic Poller // This thread samples the US and invokes
    UltrasonicPoller usPoller = null; // the selected controller on each cycle

    // Depending on which button was pressed, invoke the US poller and printer with the
    // appropriate constructor.

    switch (option) {
//      case Button.ID_LEFT: // Bang-bang control selected
//        usPoller = new UltrasonicPoller(usDistance, usData, bangbangController);
//        printer = new Printer(option, bangbangController);
//        break;
      case Button.ID_RIGHT: // Proportional control selected
        usPoller = new UltrasonicPoller(usDistance, usData, pController);
        printer = new Printer(option, pController);
        break;
      default:
        System.out.println("Error - invalid button"); // None of the above - abort
        System.exit(-1);
        break;
    }

    // Start the poller and printer threads
    usPoller.start();
    printer.start();

    // Wait here forever until button pressed to terminate wallfollower
    Button.waitForAnyPress();
    System.exit(0);

  
  }

}
