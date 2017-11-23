package ca.mcgill.ecse211.project13;

import lejos.robotics.SampleProvider;

/**
 * Control of the wall follower is applied periodically by the UltrasonicPoller thread. The while
 * loop at the bottom executes in a loop. Assuming that the us.fetchSample, and cont.processUSData
 * methods operate in about 20mS, and that the thread sleeps for 50 mS at the end of each loop, then
 * one cycle through the loop is approximately 70 mS. This corresponds to a sampling rate of 1/70mS
 * or about 14 Hz.
 */
public class UltrasonicPoller extends Thread {
  private SampleProvider us;
  private UltrasonicController cont;
  private float[] usData;
  private int distance;
  /**
   * default constructor
   * @param us sample provider used by the poller
   * @param usData float array used as buffer by the poller
   * @param cont controller used by the poller
   * 
   */
  public UltrasonicPoller(SampleProvider us, float[] usData, UltrasonicController cont) {
    this.us = us;
    this.cont = cont;
    this.usData = usData;
  }
  /**
   * another constructor
   * @param distanceMode sample provider used by the poller
   * @param usData float array used as buffer by the poller
   */
  public UltrasonicPoller(SampleProvider distanceMode, float[] usData) {
    // TODO Auto-generated constructor stub
    this.us = distanceMode;
    this.usData = usData;
  }

  /*
   * Sensors now return floats using a uniform protocol. Need to convert US result to an integer
   * [0,255] (non-Javadoc)
   * 
   * @see java.lang.Thread#run()
   */
  /**
   * Thread that runs the ultrasonic poller
   */
  public void run() {
    int distance;
    while (true) {
      us.fetchSample(usData, 0); // acquire data
      distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
      this.distance = distance;
      cont.processUSData(distance); // now take action depending on value
      try {
        Thread.sleep(50);
      } catch (Exception e) {
      } // Poor man's timed sampling
    }
  }
  /**
   * 
   * @return current distance read by the ultrasonic sensor
   */
  public int getDistance() {
    return this.distance;
  }

}
