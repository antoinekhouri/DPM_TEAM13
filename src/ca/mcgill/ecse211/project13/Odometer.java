package ca.mcgill.ecse211.project13;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
/**
 * This is the class that runs the odometer that is polled and updated by other classes.
 * The position of the robot is update based on each motor's tacho count difference.
 * @author DPM fall 2017 profs
 *
 */
public class Odometer extends Thread {
  // robot position
  private double x;
  private double y;
  private double theta;
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  private static final long ODOMETER_PERIOD = 25; /* odometer update period, in ms */

  private Object lock; /* lock object for mutual exclusion */
  /**
   * Default constructor
   * @param leftMotor robot's right motor
   * @param rightMotor robot's left motor
   */
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
  /**
   * This is where the odometer thread is run, using the run() method. 
   *
   *
   */
  // run method (required for Thread)
  public void run() {
    long updateStart, updateEnd;
    int nowTachoL;
    int nowTachoR;
    int lastTachoL;
    int lastTachoR;
    leftMotor.resetTachoCount();
    rightMotor.resetTachoCount();
    lastTachoL = leftMotor.getTachoCount();
    lastTachoR = rightMotor.getTachoCount();
    while (true) {
      double distL, distR, deltaD, deltaT, dX, dY;
      updateStart = System.currentTimeMillis();

      synchronized (lock) {
        /**
         * Don't use the variables x, y, or theta anywhere but here! Only update the values of x, y,
         * and theta in this block. Do not perform complex math
         * 
         */

        nowTachoL = leftMotor.getTachoCount();
        nowTachoR = rightMotor.getTachoCount();
        distL = (3.14159 * MainProject.WHEEL_RADIUS * (nowTachoL - lastTachoL) / 180);
        distR = (3.14159 * MainProject.WHEEL_RADIUS * (nowTachoR - lastTachoR) / 180)/1.005;
        lastTachoL = nowTachoL;
        lastTachoR = nowTachoR;
        deltaD = 0.5 * (distL + distR);
        deltaT = (distL - distR) / MainProject.TRACK;
        this.setTheta(this.theta += (deltaT * 57.2598));
        if (theta >= 360) {
          this.setTheta(theta - 360);
        }
        if (theta < 0) {
          this.setTheta(theta + 360);
        }
        dX = deltaD * Math.sin(theta / 57.2598);
        dY = deltaD * Math.cos(theta / 57.2598);
        this.setX(this.x + dX);
        this.setY(this.y + dY);


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
  /**
   * 
   * @param position double array with the robot's current position
   * @param update boolean array that indicates what coordinate to update
   */
  public void getPosition(double[] position, boolean[] update) {
    // ensure that the values don't change while the odometer is running
    synchronized (lock) {
      if (update[0])
        position[0] = x;
      if (update[1])
        position[1] = y;
      if (update[2])
        position[2] = theta;
    }
  }
  /** Returns the current X position of the robot
   * 
   * @return current X position of robot
   */
  public double getX() {
    double result;

    synchronized (lock) {
      result = x;
    }

    return result;
  }
  /**Returns the current Y position of the robot
   * 
   * @return current Y position of robot
   */
  public double getY() {
    double result;

    synchronized (lock) {
      result = y;
    }

    return result;
  }
  /** returns the current theta of the robot
   * 
   * @return current theta of robot
   */
  public double getTheta() {
    double result;

    synchronized (lock) {
      result = theta;
    }

    return result;
  }

  // mutators
  /**
   * sets the current position of the robot
   * @param position double array containing the position to set the robot at
   * @param update boolean array that indicates which coordinate to update
   */
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
  /**
   * Sets the robot's current X position
   * @param x X value to set the robot's X position to
   */
  public void setX(double x) {
    synchronized (lock) {
      this.x = x;
    }
  }
  /**
   * Sets the robot's current Y position 
   * @param y Y value to set the robot's Y position to
   */
  public void setY(double y) {
    synchronized (lock) {
      this.y = y;
    }
  }
  /**
   * Sets the robot's current theta
   * @param theta Theta value to set the robot's theta to
   */
  public void setTheta(double theta) {
    synchronized (lock) {
      this.theta = theta;
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
