package ca.mcgill.ecse211.project13;
/**
 * This is the ultrasonic controller used to enable the ultrasonic poller
 * @author DPM fall 2017 profs
 *
 */
public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
  
  public double readUSError();
}
