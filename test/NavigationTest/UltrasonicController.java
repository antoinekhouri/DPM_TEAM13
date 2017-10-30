package NavigationTest;

public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
  
  public double readUSError();
}
