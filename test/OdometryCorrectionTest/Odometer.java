package OdometryCorrectionTest;

// *** COORECT THETA! *** //

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends Thread {
  // robot position
  private double x;
  private double y;
  private double theta;
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  //
  private int nowleftMotorTachoCount;
  private int nowrightMotorTachoCount;
  public static final double WHEEL_RADIUS = 2.154;// play with values of wheel radius and Track(wheel base) until robot return to it starting position
  public static final double TRACK = 15.25;
  //
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  private static final long ODOMETER_PERIOD = 25; /*odometer update period, in ms*/

  private static Object lock; /*lock object for mutual exclusion*/

  // default constructor
  public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.x = 0.0;
    this.y = 0.0;
    this.theta = 0.0;
    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;
    lock = new Object();
  }

  // run method (required for Thread)
  
  //reference: professor odometry lecture slides 
  public void run() {
    long updateStart, updateEnd;
    double distL, distR, deltaD, deltaT, dX, dY;
    // might not need this since we're sitting them to 0 up
    leftMotor.resetTachoCount();      
    rightMotor.resetTachoCount();
    leftMotorTachoCount = leftMotor.getTachoCount();  // get tacho counts
    rightMotorTachoCount = rightMotor.getTachoCount();
    
    while (true) {
      updateStart = System.currentTimeMillis();

        nowleftMotorTachoCount = leftMotor.getTachoCount();  // get tacho counts
        nowrightMotorTachoCount = rightMotor.getTachoCount();
		distL = 3.14159*WHEEL_RADIUS*(nowleftMotorTachoCount-leftMotorTachoCount)/180;	// compute L and R wheel displacements
		distR = 3.14159*WHEEL_RADIUS*(nowrightMotorTachoCount-rightMotorTachoCount)/180;
		leftMotorTachoCount=nowleftMotorTachoCount;	// save tacho counts for next iteration
		rightMotorTachoCount=nowrightMotorTachoCount;
		deltaD = 0.5*(distL+distR);	// compute vehicle displacement
		deltaT = (distL-distR)/TRACK; // compute change in heading


      synchronized (lock) {

    	  theta += deltaT ; // update heading 
    	  dX = deltaD * Math.sin(theta); 
    	  dY = deltaD * Math.cos(theta);
    	  x += dX;
    	  y += dY;
    	  if(theta >= 2*Math.PI) { //When the value increases past 360 deg (2*pi rad)), it returns to 0
    			theta = theta - 2*Math.PI; 
    		}
    	      if(theta < 0) { //When the value decreases past 0 deg,it wraps to 359.9
    			theta = theta + (2*Math.PI);
    	    }

      }

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here because it is not
          // expected that the odometer will be interrupted by
          // another thread
        }
      }
    }
  }

  public void getPosition(double[] position, boolean[] update) {
    // ensure that the values don't change while the odometer is running
    synchronized (lock) {
      if (update[0])
        position[0] = x;
      if (update[1])
        position[1] = y;
      if (update[2])
        position[2] = getTheta();// changed theta to getTheta() to convert rad to degrees
    }
  }

  public double getX() {
    double result;

    synchronized (lock) {
      result = x;
    }

    return result;
  }

  public double getY() {
    double result;

    synchronized (lock) {
      result = y;
    }

    return result;
  }

  public double getTheta() {
    double result;

    synchronized (lock) {
    	
      result = theta*(180/3.14159); // change rad to deg
      
    }

    return result;
  }

  // mutators
  public void setPosition(double[] position, boolean[] update) {
    // ensure that the values don't change while the odometer is running
    synchronized (lock) {
      if (update[0])
        x = position[0];
      if (update[1])
        y = position[1];
      if (update[2])
        theta = position[2];
    }
  }

  public void setX(double x) {
    synchronized (lock) {
      this.x = x;
    }
  }

  public void setY(double y) {
    synchronized (lock) {
      this.y = y;
    }
  }

  public void setTheta(double theta) {
    synchronized (lock) {
      this.theta = theta ;
    }
  }

  /**
   * @return the leftMotorTachoCount
   */
  public int getLeftMotorTachoCount() {
    return leftMotorTachoCount;
  }

  /**
   * @param leftMotorTachoCount the leftMotorTachoCount to set
   */
  public void setLeftMotorTachoCount(int leftMotorTachoCount) {
    synchronized (lock) {
      this.leftMotorTachoCount = leftMotorTachoCount;
    }
  }

  /**
   * @return the rightMotorTachoCount
   */
  public int getRightMotorTachoCount() {
    return rightMotorTachoCount;
  }

  /**
   * @param rightMotorTachoCount the rightMotorTachoCount to set
   */
  public void setRightMotorTachoCount(int rightMotorTachoCount) {
    synchronized (lock) {
      this.rightMotorTachoCount = rightMotorTachoCount;
    }
  }
}