package ca.mcgill.ecse211.project13;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 20;

  private final int bandCenter;
  private final int bandwidth;
  private int distance;
  private int filterControl;
  private double error;
  /**
   * P-controller used to avoid objects by the robot
   * @param bandCenter desired minimum distance where the robot corrects its movement
   * @param bandwidth  desired threshold below which the robot starst its movement correciton 
   */
  public PController(int bandCenter, int bandwidth) {

	    this.bandCenter = bandCenter;
	    this.bandwidth = bandwidth;
	    
	    MainProject.leftMotor.setSpeed(200); // Start robot moving forward
	    MainProject.rightMotor.setSpeed(200);
	    forward();
	     
  }

  @Override
  /**
   * Ultrasonic controller default data processing method
   */
  public void processUSData(int distance) {

    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
	  double correctdistance = (distance / Math.sqrt(2));
	  error = (bandCenter - correctdistance);
	  
    if (distance >= 255  && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= 255) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }
	  //this.distance = distance;
    
    // TODO: process a movement based on the us distance passed in (P style)

    //delta = error * constant from testing + correction factor(if needed)
    double delta = ((Math.abs(error)) * 5); //+ 20 ; //relationship between speed and error/? *** find correct speed method?? ***
    int errorTooClose =  (bandCenter - bandwidth);
    int errorTooFar =  (bandCenter + bandwidth);
  
  //far from wall
    if (correctdistance > errorTooFar){
    	 int counter = 0;
    	 while (error < -bandwidth) {
    		 counter++;
    		 if (counter >= 15) {
    			 counter = 0;
    			 break;
    		 } 
    	 }
    	 
    	 MainProject.leftMotor.setSpeed((int) (MOTOR_SPEED - delta));
      	 MainProject.rightMotor.setSpeed((int) (MOTOR_SPEED + 100));
    	// MainProject.leftMotor.setAcceleration(1500);
      	 forward();
   	      
    }
    	 
    // close to wall
     //else { //if (error > bandWidth){
    else if (correctdistance < errorTooClose){
    	MainProject.leftMotor.setSpeed((int) (MOTOR_SPEED + delta));
   	    MainProject.rightMotor.setSpeed(MOTOR_SPEED);
   	    MainProject.leftMotor.setAcceleration(1500);
   	    forward();
	     
	   // System.out.println("R: " + MainProject.rightMotor.getSpeed() + " L: " + MainProject.leftMotor.getSpeed());
	    }
	  
	  //this.distance = distance;	 
    
    else {  //if (Math.abs(error) <= bandWidth){
     	MainProject.leftMotor.setSpeed(MOTOR_SPEED); // Start robot moving forward
 	    MainProject.rightMotor.setSpeed(MOTOR_SPEED);
 	    forward();
 	     
     }  
	    
  }
  /**
   * Sets both motors to go forward
   */
  public void forward(){
 	    MainProject.leftMotor.forward();
	    MainProject.rightMotor.forward();
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
  public double readUSError() {
	    return this.error;
	  }

}
