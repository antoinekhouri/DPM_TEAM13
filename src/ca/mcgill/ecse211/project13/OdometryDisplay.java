/*
 * OdometryDisplay.java
 */

package ca.mcgill.ecse211.project13;

import lejos.hardware.lcd.TextLCD;
/**
 * This class is used to display the odometry information on the LCD screen
 * @author Veronica Nasseem, Nusaiba Radi, Antoine Khouri, Nikki Daly, Diana Serra, Asma Abdullah
 *
 */
public class OdometryDisplay extends Thread {
  private static final long DISPLAY_PERIOD = 250;
  private Odometer odometer;
  private TextLCD t;

  // constructor
  /**
   * Default constructor
   * @param odometer Input odometer that is being displayed
   * @param t The textLCD object used to display the information on the robot's screen
   */
  public OdometryDisplay(Odometer odometer, TextLCD t) {
    this.odometer = odometer;
    this.t = t;
  }

  // run method (required for Thread)
  /**
   * run method for the odometry display class
   */
  public void run() {
    long displayStart, displayEnd;
    double[] position = new double[3];

    // clear the display once
    t.clear();

    while (true) {
      displayStart = System.currentTimeMillis();
      t.clear();
      // clear the lines for displaying odometry information
      t.drawString("X:              ", 0, 0);
      t.drawString("Y:              ", 0, 1);
      t.drawString("T:              ", 0, 2);
      t.drawString(MainProject.getState(), 0, 3);
      // t.drawString("" + , 0, 3);

      // get the odometry information
      odometer.getPosition(position, new boolean[] {true, true, true});

      // display odometry information
      for (int i = 0; i < 3; i++) {
        t.drawString(formattedDoubleToString(position[i], 2), 3, i);
      }
      // throttle the OdometryDisplay
      displayEnd = System.currentTimeMillis();
      if (displayEnd - displayStart < DISPLAY_PERIOD) {
        try {
          Thread.sleep(DISPLAY_PERIOD - (displayEnd - displayStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here because it is not
          // expected that OdometryDisplay will be interrupted
          // by another thread
        }
      }
    }
  }
  /**
   * formats the double values to string in order to display them
   * @param x the double value to be formatted
   * @param places Deals with decimal places
   * @return formatted input double in a string format
   */
  private static String formattedDoubleToString(double x, int places) {
    String result = "";
    String stack = "";
    long t;

    // put in a minus sign as needed
    if (x < 0.0)
      result += "-";

    // put in a leading 0
    if (-1.0 < x && x < 1.0)
      result += "0";
    else {
      t = (long) x;
      if (t < 0)
        t = -t;

      while (t > 0) {
        stack = Long.toString(t % 10) + stack;
        t /= 10;
      }

      result += stack;
    }

    // put the decimal, if needed
    if (places > 0) {
      result += ".";

      // put the appropriate number of decimals
      for (int i = 0; i < places; i++) {
        x = Math.abs(x);
        x = x - Math.floor(x);
        x *= 10.0;
        result += Long.toString((long) x);
      }
    }

    return result;
  }

}
