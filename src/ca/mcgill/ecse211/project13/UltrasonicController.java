package ca.mcgill.ecse211.project13;

public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
  
  public double readUSError();
}
