package ca.mcgill.ecse211.project13;


import ca.mcgill.ecse211.project13.UltrasonicController;	
import lejos.hardware.Sound;

/**
 * This class implements the ultrasonic localizing part of the project. It implements falling edge localization.
 * First the robot rotates clockwise until it detects a falling edge. Then it stores the 2 angles alpha1 and alpha2
 * for the first falling edge and calculates the average of the two and sets it as alpha. Then it keeps rotating 
 * clockwise until it no longer detects a wall. Then it starts rotating counter clockwise until it detects the 
 * second falling edge. It then stores beta1 and beta2, and calculates the average angle beta. Alpha and beta are 
 * then used to determine the offset of the robot, and its angle is then adjusted to the calculated angle.
 * @author Antoine Khouri, Nusaiba Radi, Veronica Nasseem, Nikki Daly, Diana Serra, Asma Abdullah
 *
 */
public class UltrasonicLocalizer implements UltrasonicController {



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

	private static boolean keepGoing=true;
	private static boolean isInPosition;

	boolean isInitialized = false;
	private   int position;
	/**
	 * Default constructor
	 * @param isFallingEdge Boolean variable indicating wether to use falling or rising edge
	 * @param odometer Odometer used to get information about the robot's current position
	 */
	public UltrasonicLocalizer(boolean isFallingEdge, Odometer odometer){
		this.isFallingEdge = isFallingEdge;
		this.odometer = odometer;

	}
	/**
	 * Performs falling edge US localization
	 * @param odometer Odometer used to get (and set) information about the robot's current position 
	 * @param distance distance read by the US sensor
	 * @throws InterruptedException 
	 */
	public static void fallingEdge(Odometer odometer, double distance) throws InterruptedException{
		MainProject.leftMotor.setSpeed(ROTATE_SPEED);
		MainProject.rightMotor.setSpeed(ROTATE_SPEED);

		//	    MainProject.leftMotor.rotate(convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, 360.0), true);
		//    	MainProject.rightMotor.rotate(-convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, 360.0), false);
		MainProject.leftMotor.forward();
		MainProject.rightMotor.backward();
		if(distance>250){			
			isInPosition = true;							    
		}
		if(isInPosition){


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
				MainProject.leftMotor.rotate(convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, 360.0), true);
				MainProject.rightMotor.rotate(-convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, 360.0), true);
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
				MainProject.leftMotor.rotate(convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, -360.0), true);
				MainProject.rightMotor.rotate(-convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, -360.0), true);
			}

			if(isAlphaSet && isBetaSet){
				if(alpha<beta){
					dTheta = -10- (alpha+beta)/2;
					isdThetaSet = true;
				} else {
					dTheta = 130- (alpha+beta)/2;
					isdThetaSet = true;
				}
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

			MainProject.rightMotor.rotate(convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, -odometer.getTheta()-dTheta), true);
			MainProject.leftMotor.rotate(-convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, -odometer.getTheta()-dTheta), false);
			if(this.position ==1 || this.position ==3){
				MainProject.rightMotor.rotate(convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, -90), true);
				MainProject.leftMotor.rotate(-convertAngle(MainProject.WHEEL_RADIUS, MainProject.TRACK, -90), false);
			}
			odometer.setTheta(0);

			keepGoing = false;
		}
	}
	public boolean getIsDone() {
		boolean isDone = !keepGoing;
		return isDone;
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
		isInPosition = false;
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
				try {
					fallingEdge(this.odometer, distance);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}	
			}

		}
		if(!isFallingEdge && keepGoing){
			if(isBetaSet && isAlphaSet){
				adjust(odometer);
			}

		}

	}
	@Override
	public int readUSDistance() {
		// 
		return 0;
	}
	@Override
	public double readUSError() {
		// TODO Auto-generated method stub
		return 0;
	}

}
