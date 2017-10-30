package ca.mcgill.ecse211.project13;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class LightLocalizer {

  public static int ROTATION_SPEED = 60;
  public static int FORWARD_SPEED = 100;
  public static int ACCELERATION = 600;
  private static final double tileLength = 30.48;
  /* knowing that the normalized lightDensity is 0(darkest)-1(brightest), 
   * we tested with different values and found that our sensor recognizes the black lines at 0.2 and below 
  */
  private static double lightDensity = 0.40;
  
  private Odometer odometer;
  private SampleProvider colorSensor;
  private float[] colorData;
  private int position;
  private int finalX;
  private int finalY;
  private int finalTheta;
 // private EV3LargeRegulatedMotor MainProject.leftMotor, MainProject.rightMotor;
  /**
   * Default constructor
   * @param odometer Odometer used to get information about robot's current position
   * @param colorSensor	SampleProvider used by the light sensor
   * @param colorData Buffer used to store the light sensor's data
   */
  public LightLocalizer(Odometer odometer, SampleProvider colorSensor, float[] colorData)
		  	//EV3LargeRegulatedMotor MainProject.leftMotor, EV3LargeRegulatedMotor MainProject.rightMotor) 
		  {
    this.odometer = odometer;
    this.colorSensor = colorSensor;
    this.colorData = colorData;
    //this.MainProject.leftMotor = MainProject.leftMotor;
    //this.MainProject.rightMotor = MainProject.rightMotor;

    MainProject.leftMotor.setAcceleration(ACCELERATION);
    MainProject.rightMotor.setAcceleration(ACCELERATION);

  }

  //Localize robot using the light sensor
  /**
   * Localizes the robot the the desired grid line intersection
   * @param position 0-1-2-3, the position on the grid where the robot is placed at
   */
  public void localize(int position) {
	this.position = position;
	if(position == 0){
		this.finalX = 1;
		this.finalY = 1;
		this.finalTheta =0;
	}
	else if(position ==1){
		this.finalX = 7;
		this.finalY = 1;
		this.finalTheta = 0;
	}
	else if(position == 2){
		this.finalX = 7;
		this.finalY = 7;
		this.finalTheta = 180;
	}
	else if(position ==3){
		this.finalX = 1;
		this.finalY = 7;
		this.finalTheta = 180;
	}
    // set the speeds of the motors and move forward (in the y direction)
    MainProject.leftMotor.setSpeed(FORWARD_SPEED);
    MainProject.rightMotor.setSpeed(FORWARD_SPEED);
    MainProject.leftMotor.forward();
    MainProject.rightMotor.forward();

    //keep going forward until we hit a black line
    while (getColorData() > lightDensity) {
    		//try-catch from ultrasonic poller
            try {
        Thread.sleep(100);
      } catch (InterruptedException e) {
        //Auto-generated catch block
        e.printStackTrace();
      }
    }
    Sound.beep();
    //reached a black line so stop
    MainProject.leftMotor.stop(true);
    MainProject.rightMotor.stop(true);
    
    // now go backwards the same y distance that was moved forward to reach a black line
    double ySawLine = odometer.getY();
    MainProject.leftMotor.rotate(-convertDistance(MainProject.WHEEL_RADIUS, Math.abs(ySawLine)), true); 
    MainProject.rightMotor.rotate(-convertDistance(MainProject.WHEEL_RADIUS, Math.abs(ySawLine)), false);
    
    // now rotate 90 degrees clockwise (assuming we started at 0 degrees from the ultrasonicLocalizer) 
    //to do the same thing in the x direction
    MainProject.leftMotor.setSpeed(ROTATION_SPEED);
    MainProject.rightMotor.setSpeed(ROTATION_SPEED);
    if(position == 0 || position == 2){
    	MainProject.leftMotor.rotate(convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, 90.0), true);
    	MainProject.rightMotor.rotate(-convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, 90.0), false);
    }
    else if(position ==1 || position == 3){
    	MainProject.leftMotor.rotate(convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, -90.0), true);
    	MainProject.rightMotor.rotate(-convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, -90.0), false);
    }

    // set the speeds of the motors and move forward (in the x direction)
    MainProject.leftMotor.setSpeed(FORWARD_SPEED);
    MainProject.rightMotor.setSpeed(FORWARD_SPEED);
    MainProject.leftMotor.forward();
    MainProject.rightMotor.forward();

    // keep going forward until we hit a black line
    while (getColorData() >  lightDensity) { 
		//try-catch from ultrasonic poller
    	
      try {
        Thread.sleep(100);
      } catch (InterruptedException e) {
        //Auto-generated catch block
        e.printStackTrace();
      }
    }
    Sound.beep();
    //reached a black line so stop
    MainProject.leftMotor.stop(true);
    MainProject.rightMotor.stop(true);
    
    // now go backwards the same x distance that was moved forward to reach a black line
    double xSawLine = odometer.getX();
    
    MainProject.leftMotor.rotate(-convertDistance(MainProject.WHEEL_RADIUS, Math.abs(xSawLine)), true); 
    MainProject.rightMotor.rotate(-convertDistance(MainProject.WHEEL_RADIUS, Math.abs(xSawLine)), false);

    // rotate -45 degrees (i.e: counterclockwise) to face the (0,0) point
    double angle1 = Math.toDegrees(Math.atan(ySawLine/xSawLine));
    if(position ==0 || position ==2){
    	MainProject.leftMotor.rotate(-convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, angle1), true);
    	MainProject.rightMotor.rotate(convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, angle1), false);
    }
    else if (position ==1 || position ==3){
    	MainProject.leftMotor.rotate(-convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, angle1), true);
    	MainProject.rightMotor.rotate(convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, angle1), false);
    }
    
    //travel to (0,0) 
    //hypotenuse of the triangle with x and y being the distances from the 2 black lines detected
    //distance to travel is hypotenuse PLUS the measured distance between the lightsensor and our centre of rotation
    double distance = Math.sqrt((Math.pow(xSawLine, 2) + Math.pow(ySawLine, 2)));
    double sensorToCentre = 9;
    MainProject.leftMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, distance+sensorToCentre), true); 
    MainProject.rightMotor.rotate(convertDistance(MainProject.WHEEL_RADIUS, distance+sensorToCentre), false);

    // rotate another -45 degrees (i.e: counterclockwise) to face 0 degrees
    
    if(position ==0 || position ==2){
    	MainProject.leftMotor.rotate(-convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, 90-angle1), true);
    	MainProject.rightMotor.rotate(convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, 90-angle1), false);
    }else if(position ==1 || position ==3){
    	MainProject.leftMotor.rotate(-convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, -(90+angle1)), true);
    	MainProject.rightMotor.rotate(convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, -(90+angle1)), false);
    }

    //correct the x and y readings to zeroes
    odometer.setX(finalX*tileLength);
    odometer.setY(finalY*tileLength);
    odometer.setTheta(finalTheta);

  }

  // conversion methods
  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  // gets the data from the color sensor, and returns a value corresponding
  // to the overall "brightness" which is the avg of magnitudes of red, green, and blue
  private float getColorData() {
    colorSensor.fetchSample(colorData, 0);
    float colorBrightnessLevel = (colorData[0] + colorData[1] + colorData[2]);
    return colorBrightnessLevel;
  }


}