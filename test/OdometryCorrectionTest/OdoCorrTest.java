package OdometryCorrectionTest;

import NavigationTest.Navigation;
import NavigationTest.UltrasonicPoller;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;


public class OdoCorrTest extends Thread{


	  private static final long CORRECTION_PERIOD = 10;
	  private Odometer odometer;
	  
	  //initializing color/light sensor
	  static Port lightSensor = LocalEV3.get().getPort("S1");
	  static SensorModes ls = new EV3ColorSensor(lightSensor);
	  static SampleProvider myColorSample = ls.getMode("Red"); 
	  static float[] sampleColor = new float[ls.sampleSize()];
	  
	  //counter for black lines (along x and y axes)
	  int xCounter = -1; 
	  int yCounter = -1;

	  // constructor
	  public OdoCorrTest(Odometer odometer) {
	    this.odometer = odometer;
	  }

	  public OdoCorrTest(NavigationTest.Odometer odometer2, SampleProvider colorValueLeft, float[] colorDataLeft,
			SampleProvider colorValueRight, float[] colorDataRight, Navigation nav, UltrasonicPoller usPoller,
			SampleProvider usDistance) {
		
	}

	// run method (required for Thread)
	  public void run() {
	    long correctionStart, correctionEnd;

	    while (true) {
	      correctionStart = System.currentTimeMillis();
	      
	      //get the sample color/light from the sensor
	      myColorSample.fetchSample(sampleColor, 0); 
	      
	      //store light intensity for comparison
	      double lightDensity =  (sampleColor[0]);      
	      
	      //after testing, found that sensor recognizes the black lines when the light intensity is less than 0.4
	      if (lightDensity < 0.4) { 
	    	  
		    	  Sound.beep(); // beep when cross black line
		    	  
		    	  double theta = odometer.getTheta(); //get theta in deg for comparison
		    	  double tileWidth = 30.48;
		
		    	//theta ranged from 350 to 10 degrees while along the first axis (y in our case) during tests
		    	//along
		    	  if(theta >= 350 && theta < 359.99 || theta >= 0 && theta <10 ){ 
		    		  
		    		  yCounter++; // increment y to 0 when crossing first line, 1 for the second etc..
		    		  odometer.setY(yCounter * tileWidth); // correct y distance traversed (0 - first line, 30.48 - second line,etc..)
		    	  }
		    	  //second axis (x)
		    	  else if(theta >= 85 && theta < 95){
		    		  xCounter++;
		    		  odometer.setX(xCounter * tileWidth);
		    	  }
		    	  //third axis (y)
		    	  else if(theta >= 175 && theta < 185){
		    		  odometer.setY(yCounter * tileWidth);
		    		  yCounter--; //decrement counter as we are now going back towards the point we started
		    		  // note that 'decrementation' of the counter is AFTER correcting y position
		    	  }
		    	  //final axis (x)
		    	  else{
		    		  odometer.setX(xCounter * tileWidth);
		    		  xCounter--;
		    	  }
	      }

	      // this ensure the odometry correction occurs only once every period
	      correctionEnd = System.currentTimeMillis();
	      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
	        try {
	          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
	        } catch (InterruptedException e) {
	          // there is nothing to be done here because it is not
	          // expected that the odometry correction will be
	          // interrupted by another thread
	        }
	      }
	    }
	  }
	
}
