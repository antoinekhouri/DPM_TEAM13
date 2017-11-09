package LocalizationTest;


import LocalizationTest.UltrasonicController;
import lejos.hardware.Sound;


public class USLocalizationTest implements UltrasonicController {

	public static double distance;
	private static final int ROTATE_SPEED = 60;
	private boolean isFallingEdge;
	private Odometer odometer;
	private static int k = 1;
	private static int d = 30;
	private static boolean isAlphaOneSet;
	private static boolean isAlpha2Set;
	private static boolean isAlphaSet;
	private static boolean isFirstTurnDone;
	private static double alpha1;
	private static double alpha2;
	private static double alpha;
    
	private static boolean isBeta1Set;
	private static boolean isBeta2Set;
	private static boolean isBetaSet;
	private static boolean isdThetaSet;
	private static double beta1;
	private static double beta2;
	private static double beta;
	
	private static double minTheta;
    private static double minDistance;
	
	private static double dTheta;
	
	private static boolean keepGoing;
	private static boolean isInPosition;
	
	boolean isInitialized = false;
	private   int position;
	/**
	 * Default constructor
	 * @param isFallingEdge Boolean variable indicating wether to use falling or rising edge
	 * @param odometer Odometer used to get information about the robot's current position
	 * @param position Position the robot is placed at (0-1-2-3)
	 */
	public USLocalizationTest(boolean isFallingEdge, Odometer odometer){
		this.isFallingEdge = isFallingEdge;
		this.odometer = odometer;
		this.position = position;
	}
	/**
	 * Performs falling edge US localization
	 * @param odometer Odometer used to get (and set) information about the robot's current position 
	 * @param distance distance read by the US sensor
	 */
	public static void fallingEdge(Odometer odometer, double distance){
		LocalizationTestMain.leftMotor.setSpeed(ROTATE_SPEED);
	    LocalizationTestMain.rightMotor.setSpeed(ROTATE_SPEED);
	    
//	    LocalizationTestMain.leftMotor.rotate(convertAngle(LocalizationTestMain.WHEEL_RADIUS, LocalizationTestMain.TRACK, 360.0), true);
//    	LocalizationTestMain.rightMotor.rotate(-convertAngle(LocalizationTestMain.WHEEL_RADIUS, LocalizationTestMain.TRACK, 360.0), false);

	    if(distance<d+k && !isAlphaOneSet){
	    	alpha1 = odometer.getTheta();
	    	Sound.beep();
	    	isAlphaOneSet = true;
	    }
	    
	    if(distance<d-k && isAlphaOneSet && !isAlpha2Set){
	    	alpha2 = odometer.getTheta();
	    	Sound.beep();
	    	isAlpha2Set = true;
	    }
	      
	    if(isAlphaOneSet && isAlpha2Set){
	    	alpha = (alpha1 + alpha2)/2;
	    	isAlphaSet = true;
	    }
	    
	    if(isAlphaSet && distance>250){
	    	isFirstTurnDone = true;
	    	
	    } else {
	    	LocalizationTestMain.leftMotor.rotate(convertAngle(LocalizationTestMain.WHEEL_RADIUS, LocalizationTestMain.TRACK, 360.0), true);
	    	LocalizationTestMain.rightMotor.rotate(-convertAngle(LocalizationTestMain.WHEEL_RADIUS, LocalizationTestMain.TRACK, 360.0), true);
	    }
	      
	    if(distance<d+k && isFirstTurnDone && !isBeta1Set){
	    	beta1 = odometer.getTheta();
	    	isBeta1Set = true;
	    	Sound.beep();
	    }
	      
	    if(distance<d-k && isBeta1Set && !isBeta2Set){
	    	beta2 = odometer.getTheta();
	    	isBeta2Set = true;
	    	Sound.beep();
	    }
	   	  
	    if(isBeta1Set && isBeta2Set){
	   		beta = (beta1+beta2)/2;
	       isBetaSet = true;
	       
	   	}
	   
	    if(isFirstTurnDone && !isBetaSet){
	    	LocalizationTestMain.leftMotor.rotate(convertAngle(LocalizationTestMain.WHEEL_RADIUS, LocalizationTestMain.TRACK, -360.0), true);
	    	LocalizationTestMain.rightMotor.rotate(-convertAngle(LocalizationTestMain.WHEEL_RADIUS, LocalizationTestMain.TRACK, -360.0), true);
	    }
	    
	    if(isAlphaSet && isBetaSet){
	    	if(alpha<beta){
	    		dTheta = -35- (alpha+beta)/2;
	    		isdThetaSet = true;
	    	} else {
	    		dTheta = 130- (alpha+beta)/2;
	   			isdThetaSet = true;
	   		  }
	   	 }	   	  	   	  
		}
	
	/**
	 * Sets the robot to the correct 0-degree angle
	 * @param odometer Odometer used to get and set information about robot's position
	 */
	public  void adjust(Odometer odometer){
	  
  	  if(isdThetaSet){
  		  while (dTheta>360){
  			  dTheta = dTheta - 360;
  		  }
  		  
  		  LocalizationTestMain.rightMotor.rotate(convertAngle(LocalizationTestMain.WHEEL_RADIUS, LocalizationTestMain.TRACK, -odometer.getTheta()-dTheta), true);
  		  LocalizationTestMain.leftMotor.rotate(-convertAngle(LocalizationTestMain.WHEEL_RADIUS, LocalizationTestMain.TRACK, -odometer.getTheta()-dTheta), false);
  		  if(this.position ==1 || this.position ==3){
  			  LocalizationTestMain.rightMotor.rotate(convertAngle(LocalizationTestMain.WHEEL_RADIUS, LocalizationTestMain.TRACK, -90), true);
  	  		  LocalizationTestMain.leftMotor.rotate(-convertAngle(LocalizationTestMain.WHEEL_RADIUS, LocalizationTestMain.TRACK, -90), false);
  		  }
  		  odometer.setTheta(0);
  		  keepGoing = false;
  	  }
	}
	// initialize all the global variables
	/**
	 * initalize all the global variable
	 */
	private void initialize(){
		isAlphaOneSet = false;
		isAlpha2Set = false;
		isAlphaSet = false;
		isFirstTurnDone = false;
		alpha1 = 0;
		alpha2 = 0;
		alpha = 0;
		
		minTheta =  10;
		minDistance = 250;
		keepGoing = true;
	    
		isBeta1Set = false;
		isBeta2Set = false;
		isBetaSet = false;
		isdThetaSet = false;
		beta1 = 0;
		beta2 = 0;
		beta = 0;
	    
		
		dTheta = 0;
		isInPosition = false;
	}
	/**
	 * Performs rising edge ultrasonic localization
	 * @param odometer Odometer used to get and set information about the robot's position
	 * @param distance Distance read by the US sensor
	 */
	public static void risingEdge(Odometer odometer, double distance){
		  LocalizationTestMain.leftMotor.setSpeed(ROTATE_SPEED);
	      LocalizationTestMain.rightMotor.setSpeed(ROTATE_SPEED);
	      
	      if(distance > d-k && !isAlphaOneSet){
	    	  alpha1 = odometer.getTheta();
	    	  isAlphaOneSet = true;
	      }
	      
	      if(distance>d+k && isAlphaOneSet && !isAlpha2Set){
	    	  alpha2 = odometer.getTheta();
	    	  isAlpha2Set = true;
	      }
	      
	      if(isAlphaOneSet && isAlpha2Set){
	    	  alpha = (alpha1 + alpha2)/2;
	    	  isAlphaSet = true;
	      }
	      //Turn until the first rising edge is detected
	      if(isAlphaSet && distance>minDistance){
	    	  isFirstTurnDone = true;
	      } else {
		      LocalizationTestMain.leftMotor.rotate(convertAngle(LocalizationTestMain.WHEEL_RADIUS, LocalizationTestMain.TRACK, 360.0), true);
	    	  LocalizationTestMain.rightMotor.rotate(-convertAngle(LocalizationTestMain.WHEEL_RADIUS, LocalizationTestMain.TRACK, 360.0), true);
	      }
	      
	      if(isFirstTurnDone && odometer.getTheta()<minTheta){
	    	  isInPosition =true;
	      }
	      
	      if(distance>d-k && isFirstTurnDone && !isBeta1Set && isInPosition){
	    	  beta1 = odometer.getTheta();
	    	  isBeta1Set = true;
	      }
	      
	      if(distance>d+k && isBeta1Set && !isBeta2Set){
	    	  beta2 = odometer.getTheta();
	    	  isBeta2Set = true;
	      }
	      
	   	  if(isBeta1Set && isBeta2Set){
	   		  beta = (beta1+beta2)/2;
	   		  isBetaSet = true;
	   	  }
	   	  
	   	  //turn until the second rising edge is detected
	   	  if(isFirstTurnDone && !isBetaSet){
	    	  LocalizationTestMain.leftMotor.rotate(convertAngle(LocalizationTestMain.WHEEL_RADIUS, LocalizationTestMain.TRACK, -360.0), true);
	    	  LocalizationTestMain.rightMotor.rotate(-convertAngle(LocalizationTestMain.WHEEL_RADIUS, LocalizationTestMain.TRACK, -360.0), true);
	      }

	   	  if(isAlphaSet && isBetaSet){
	   		  if(alpha<beta){
	   			  dTheta = 115- (alpha+beta)/2;
	   			  isdThetaSet = true;
	   		  }
	   		  else{
	   			  dTheta = 295- (alpha+beta)/2;
	   			  isdThetaSet = true;
	   		  }
	   	  }	  
	}
	
	//methods borrowed from pervious lab
	private static int convertDistance(double radius, double distance) {
	   return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	
    private static int convertAngle(double radius, double width, double angle) {
    	return convertDistance(radius, Math.PI * width * angle / 360.0);
    }
    
    
	@Override	
	/**
	 * Finite state machine based on rising/falling edge selection & US sensor distance read
	 */
	public void processUSData(int distance) {
		
		if(!isInitialized){
//			if(distance>200){
//				this.isFallingEdge = true;
//			}
//			else{
//				this.isFallingEdge = false;
//			}
			initialize();
			isInitialized = true;
		}
		
		this.distance = distance;
		if(isFallingEdge && keepGoing){
			if(isBetaSet && isAlphaSet){
				
				adjust(odometer);
				
			}else{
				fallingEdge(this.odometer, distance);	
			}
			
		}
		if(!isFallingEdge && keepGoing){
			if(isBetaSet && isAlphaSet){
				adjust(odometer);
			}
			else{
				risingEdge(this.odometer, distance);
			}
		}
		
	}
	@Override
	public int readUSDistance() {
		// 
		return 0;
	}
	
}
