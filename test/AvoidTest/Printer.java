package AvoidTest;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import NavigationTest.UltrasonicPoller;
import NavigationTest.UltrasonicController;

public class Printer extends Thread {

  //
  // In addition to the UltrasonicPoller, the printer thread also operates
  // in the background. Since the thread sleeps for 200 mS each time through
  // the loop, screen updating is limited to 5 Hz.
  //

  private UltrasonicController cont;
  private final int option;
  /**
   * Default constructor
   * @param option used to select P or bang-bang controller type
   * @param cont ultrasonic controller used to process data from the US sensor
   */
  public Printer(int option, UltrasonicController cont) {
    this.cont = cont;
    this.option = option;
  }

  public static TextLCD t = LocalEV3.get().getTextLCD(); // n.b. how the screen is accessed
  /**
   * Run method that runs the printer thread
   */
  public void run() {
    while (true) { // operates continuously
      t.clear();
      t.drawString("Controller Type is... ", 0, 0); // print header
      if (this.option == Button.ID_LEFT)
        t.drawString("BangBang", 0, 1);
      else if (this.option == Button.ID_RIGHT)
        t.drawString("P type", 0, 1);
      t.drawString("US Distance: " + cont.readUSDistance(), 0, 2); // print last US reading
      t.drawString("Error: " + cont.readUSError(), 0, 4); // print last US reading

      try {
        Thread.sleep(200); // sleep for 200 mS
      } catch (Exception e) {
        System.out.println("Error: " + e.getMessage());
      }
    }
  }
  /**
   * Prints the main menu screen
   */
  public static void printMainMenu() { // a static method for drawing
    t.clear(); // the screen at initialization
    t.drawString("left = bangbang", 0, 0);
    t.drawString("right = p type", 0, 1);
  }
}
