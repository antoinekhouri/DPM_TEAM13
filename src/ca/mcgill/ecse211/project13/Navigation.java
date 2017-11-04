package ca.mcgill.ecse211.project13;

import java.util.Set;	
import ca.mcgill.ecse211.project13.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
/**
 * This class is what is used to navigate the robot to (x,y) and align it with (Xc, Yc). 
 * The robot first moves up the X axis, then turns 90 degrees and moves up the Y axis in this class.
 * @author Veronica Nasseem, Nusaiba Radi, Antoine Khouri, Nikki Daly, Diana Serra, Asma Abdullah
 *
 */
public class Navigation implements UltrasonicController {
  // Constants and variables
  private static final int FORWARD_SPEED = 185;
  private static final int FORWARD_SPEED_Right = 185;
  private static final int ROTATE_SPEED = 50;
  private static final double tileLength = 30.48;

  private Odometer odometer;
  private static final Port usPort = LocalEV3.get().getPort("S2");
  private static final double pulley_to_roborCenter = 1.3;
  private boolean isAvoidingWall;
  private double wallDistance;
  private double theta;


  /** 
   * This constructor is simply used in order to indicate to the system whether or not it will
   * require wall avoidance.
   * @param isAvoidingWall a boolean that indicates whether or not the system will require to avoid a wall
   *
   */
  public Navigation(boolean isAvoidingWall) {
    this.isAvoidingWall = isAvoidingWall;
  }
  /**
   * This method calculates the minimum angle to turn in order to face the destination,
   * then uses the turnTo() method in order to turn to said angle, then calculates the distance required
   * in order to arrive to the destination and finally it travels that distance
   * @param usDistance SampleProvider used by the ultrasonic poller
   * @param odometer Odometer used to get info about the current position and orientation of the robot
   * @param leftRadius Double representing the left wheel radius, used for the wheel movement
   * @param rightRadius Double representing the right wheel radius, used for the wheel movement
   * @param width Double representing the distance between the two wheels, used for the robot movement
   * @param x0 destination X coordinate
   * @param y0 destination Y coordinate
   * @param xC zip line start X coordinate
   * @param yC zip line start y coordinate
   */
  public void travelTo(SampleProvider usDistance, Odometer odometer, double leftRadius,
      double rightRadius, double width, double x0, double y0, double xC, double yC) {


    double xDistance = x0 * tileLength - odometer.getX();
    MainProject.leftMotor.setSpeed(ROTATE_SPEED);
    MainProject.rightMotor.setSpeed(FORWARD_SPEED_Right);

    if (Math.abs(xDistance) < 10) { // If the destination is on the same X axis, do not rotate
      xDistance = 0;
    } else if (x0 * tileLength > odometer.getX()) { // If the destination is to the right of the
                                                    // current position, rotate 90 degrees clockwise
      turnTo(90, odometer, leftRadius, rightRadius, width);
    } else if (x0 * tileLength < odometer.getX()) { // Otherwise, rotate 90 degrees counter clock
                                                    // wise
      turnTo(270, odometer, leftRadius, rightRadius, width);
    }

    MainProject.leftMotor.setSpeed(FORWARD_SPEED);
    MainProject.rightMotor.setSpeed(FORWARD_SPEED_Right);

    MainProject.leftMotor.rotate(convertDistance(leftRadius, Math.abs(xDistance)), true);
    MainProject.rightMotor.rotate(convertDistance(rightRadius, Math.abs(xDistance)), false);

    MainProject.leftMotor.setSpeed(ROTATE_SPEED);
    MainProject.rightMotor.setSpeed(FORWARD_SPEED_Right);

    // Same rotation logic as on the x axis
    double yDistance = y0 * tileLength - odometer.getY();

    if (Math.abs(yDistance) < 10) {
      yDistance = 0;
    } else if (y0 * tileLength > odometer.getY()) {
      turnTo(0, odometer, leftRadius, rightRadius, width);
    } else if (y0 * tileLength < odometer.getY()) {
      turnTo(180, odometer, leftRadius, rightRadius, width);
    }
    // -1.3 since our pulley is not exactly at the robot center
    yDistance = yDistance - pulley_to_roborCenter;
    MainProject.leftMotor.setSpeed(FORWARD_SPEED);
    MainProject.rightMotor.setSpeed(FORWARD_SPEED_Right);

    MainProject.leftMotor.rotate(convertDistance(leftRadius, Math.abs(yDistance)), true);
    MainProject.rightMotor.rotate(convertDistance(rightRadius, Math.abs(yDistance)), false);

    if ((Math.abs(xC * tileLength - odometer.getX())) < 10) {
      if (yC * tileLength - odometer.getY() > 0) {
        theta = 0;
        turnTo(theta, odometer, leftRadius, rightRadius, width);
        MainProject.leftMotor.stop(true);
        MainProject.rightMotor.stop(true);
      } else {
        theta = 180;
        turnTo(theta, odometer, leftRadius, rightRadius, width);
        MainProject.leftMotor.stop(true);
        MainProject.rightMotor.stop(true);
      }
    } else if (Math.abs((yC * tileLength - odometer.getY())) < 10) {
      if (xC * tileLength > odometer.getX()) {
        theta = 90;
        turnTo(theta, odometer, leftRadius, rightRadius, width);
        MainProject.leftMotor.stop(true);
        MainProject.rightMotor.stop(true);
      } else {
        theta = 270;
        turnTo(theta, odometer, leftRadius, rightRadius, width);
        MainProject.leftMotor.stop(true);
        MainProject.rightMotor.stop(true);
      }

    } else {

      // if the x and y error is large it will change the angle of the robot
      if (yC * tileLength > odometer.getY()) {
        turnTo(
            Math.toDegrees(Math
                .atan((xC * tileLength - odometer.getX()) / (yC * tileLength - odometer.getY()))),
            odometer, leftRadius, rightRadius, width);
        MainProject.leftMotor.stop(true);
        MainProject.rightMotor.stop(true);
      } else if (xC * tileLength < odometer.getX()) {
        turnTo((-1)
            * Math.toDegrees(Math
                .atan((yC * tileLength - odometer.getY()) / (xC * tileLength - odometer.getX())))
            - 90, odometer, leftRadius, rightRadius, width);
        MainProject.leftMotor.stop(true);
        MainProject.rightMotor.stop(true);
      } else {
        turnTo((-1)
            * Math.toDegrees(Math
                .atan((yC * tileLength - odometer.getY()) / (xC * tileLength - odometer.getX())))
            + 90, odometer, leftRadius, rightRadius, width);
        MainProject.leftMotor.stop(true);
        MainProject.rightMotor.stop(true);
      }

    }
  }

  /**
   * 
   * @param theta the angle the robot should turn to
   * @param odometer the odometer used to get information about robot's current position/orientation
   * @param leftRadius Double representing the left wheel radius, used for the wheel movement
   * @param rightRadius Double representing the right wheel radius, used for the wheel movement
   * @param width Double representing the distance between the two wheels, used for the robot movement
   */
  // Helper method to turn the robot to a given angle
  public void turnTo(double theta, Odometer odometer, double leftRadius, double rightRadius,
      double width) {

    MainProject.leftMotor.setSpeed(ROTATE_SPEED);
    MainProject.rightMotor.setSpeed(ROTATE_SPEED);
    while (theta - odometer.getTheta() > 360) {
      theta = theta - 360;
    }
    while (theta - odometer.getTheta() < 0) {
      theta = theta + 360;
    }
    if ((theta - odometer.getTheta()) > 180) {

      MainProject.rightMotor
          .rotate(convertAngle(rightRadius, width, 360 - (theta - odometer.getTheta())), true);
      MainProject.leftMotor
          .rotate((-convertAngle(leftRadius, width, 360 - (theta - odometer.getTheta()))), false);
    } else {
      MainProject.leftMotor.rotate((convertAngle(leftRadius, width, theta - odometer.getTheta())),
          true);
      MainProject.rightMotor.rotate(-convertAngle(rightRadius, width, theta - odometer.getTheta()),
          false);
    }

  }
  /**
   * Converts the distance so that it can be used by the robot
   * @param radius wheel radius
   * @param distance distance the robot should move
   * @return converted distance for the motors to use
   */
  // Conversion method
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }
  /**
   * Converts angle so that it can be used by the robot to rotate
   * @param radius wheel radius
   * @param width distance between the two wheels
   * @param angle angle the robot should turn 
   * @return
   */
  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

  // helper method to read usData
  @Override
  public void processUSData(int distance) {
    this.wallDistance = distance;
  }

  @Override
  public int readUSDistance() {
    return 0;
  }

@Override
public double readUSError() {
	// TODO Auto-generated method stub
	return 0;
}

}
