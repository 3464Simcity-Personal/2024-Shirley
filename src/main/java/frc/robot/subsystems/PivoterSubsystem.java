// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivoterConstants;

@SuppressWarnings("removal")
public class PivoterSubsystem extends SubsystemBase {
  // Create motor and limit switch. 
  private final CANSparkMax pivoterMotor = new CANSparkMax(PivoterConstants.kPivoterMotorPort, MotorType.kBrushless); //Right Motor for Arm Pivoter
  private final DigitalInput minLimitSwitch = new DigitalInput(PivoterConstants.kPivotMinSwitchPort);
  private final CANSparkMax secondPivoterMotor = new CANSparkMax(PivoterConstants.kPivoterSecondMotorPort, MotorType.kBrushless);

  // Get the encoder from the motor. 
  private final RelativeEncoder pivoterEncoder = pivoterMotor.getEncoder(); //Encoder for Arm Pivoter Left Motor Position (used for both)
  
  private final SparkMaxPIDController m_pidController;

  private Gains gains = new Gains(1.5e-4, 3e-6, 0.000156, 0, 1, 0.75); //smart motion gains

  private static final int SMART_MOTION_SLOT = 0;

  // SmartMotion configs
  private static final double MAX_VELOCITY_RPM = 5_676; // NEO free speed 5676 RPM
  private static final double MIN_VELOCITY_RPM = 0;
  private static final double MAXX_ACCELERATION_RPM_PER_SEC = 30_000;
  private static final double ALLOWED_ERROR = 0.1; //motor rotations

  private static final double GEAR_RATIO = (5.0/1.0) * (3.0/1.0) * (54.0 / 22.0) * (74.0 / 16.0);
  private static final double DEGREES_PER_REV = 360.0;
  private static final int CURRENT_LIMIT = 13; //Amps
  private static final boolean INVERT_MOTOR = true;

  // Voltage needed to maintain horizontal arm position.
  private static final double horizontalArbFF = 0.30;

  public PivoterSubsystem() {
    pivoterMotor.setInverted(false);
    // secondPivoterMotor.setInverted(false);
    secondPivoterMotor.follow(pivoterMotor, true); // This inversts the motor and tells it to follow the other. 

    m_pidController = pivoterMotor.getPIDController();

    // Set PID coefficients
    m_pidController.setP(gains.kP, SMART_MOTION_SLOT);
    m_pidController.setI(gains.kI, SMART_MOTION_SLOT);
    m_pidController.setD(gains.kD, SMART_MOTION_SLOT);
    m_pidController.setIZone(gains.kIzone, SMART_MOTION_SLOT);
    m_pidController.setFF(gains.kF, SMART_MOTION_SLOT);
    m_pidController.setOutputRange(-gains.kPeakOutput, gains.kPeakOutput, SMART_MOTION_SLOT);

    m_pidController.setSmartMotionMaxVelocity(MAX_VELOCITY_RPM, SMART_MOTION_SLOT);
    m_pidController.setSmartMotionMinOutputVelocity(MIN_VELOCITY_RPM, SMART_MOTION_SLOT);
    m_pidController.setSmartMotionMaxAccel(MAXX_ACCELERATION_RPM_PER_SEC, SMART_MOTION_SLOT);
    m_pidController.setSmartMotionAllowedClosedLoopError(ALLOWED_ERROR, SMART_MOTION_SLOT);
        
  }

 /*
  * Pivoter Motor methods. 
  */

  private double degreesToMotorRotations(double degrees) {
    // return degrees/DEGREES_PER_REV * GEAR_RATIO;
    return degrees / PivoterConstants.kPivoterRotationToDegree;
  }

  public double motorRotationsToDegrees(double rotations) {
    // return (rotations / GEAR_RATIO) * DEGREES_PER_REV;
    return rotations * PivoterConstants.kPivoterRotationToDegree;
  }

    //  Run the motor to our inputted degrees. 
  public void pivot(double rotations) {
    m_pidController.setReference(
      rotations,
      // CANSparkMax.ControlType.kPosition,
      CANSparkMax.ControlType.kSmartMotion,
      SMART_MOTION_SLOT,
      getArbFF()
    );
  }

    /**
   * Get the Arbitrary Feed-Forward term, voltage needed to maintain arm positon. 

   * @return Arbitrary Feed-Forward (Volts)
   */
  public double getArbFF() {
    double degrees = getPositionDegrees();
    double radians = Math.toRadians(degrees);

    return Math.cos(radians) * horizontalArbFF;
  }

  public double getPositionDegrees() {
    return motorRotationsToDegrees(pivoterEncoder.getPosition());
  }


  // Stop motor. 
  public void stopMotor(){
    pivoterMotor.stopMotor();
    secondPivoterMotor.stopMotor();
  }

  // public void pivotForward(){
  //   pivot(0.125);
  // }

  // public void pivotToMin(){
  //   pivot(-0.25); // Run pivot continously till we hit the switch, which it should do. 
  // }

  // public Command pivotManual(double speed) {
  //   // Manual pivot command
  //   return runOnce(
  //       () -> {
  //         if(minLimitSwitch.get() && (Math.signum(speed) < 0)){ // if the min limit switch is triggered and we're trying to go down. 
  //           pivoterMotor.stopMotor();
  //           pivoterEncoder.setPosition(0);
  //         }else{
  //           pivoterMotor.set(speed); // Else, run the speed we want to set. 
  //         }
  //       });
  // }


  /*
   * Pivoter Encoder methods. 
   */

  public double getPivoterSpeed() {
    // Get the speed of the motor. 
    return pivoterMotor.get();
  }

  public double getPivoterRotation(){
    // Return the pivoter position in rotations. 
    return pivoterEncoder.getPosition();
  }

  public double getPivoterDegrees(){
    // Multiply the position by the conversion factor that changes it from rotation to degrees. 
    return pivoterEncoder.getPosition() * PivoterConstants.kPivoterRotationToDegree; 
  }
  
  public void resetEncoder(){
    // Set the encoder back to normal
    pivoterEncoder.setPosition(0);
  }

  public void addFeedFoward(){
    // Add some power to the pivoter to have it hold against gravity. 
    if(!getSwitch()){ // Make sure the trigger isn't activated. 
      //System.out.println("FeedFoward");  
//       pivot(0.05);
      // secondPivoterMotor.fo
    }
  }

  /*
   * Limit switch commands. 
   */

   public boolean getSwitch(){
    // Get the limit switch - either true or false. 
    //System.out.print(!minLimitSwitch.get());
    return !minLimitSwitch.get();
    
   }

  @Override
  public void periodic() {
    // Print out pivoter degrees and speed
    // SmartDashboard.putBoolean("Motor Connected", pivoterMotor.con)
    SmartDashboard.putNumber("Pivoter Degrees:", getPivoterDegrees()); // get the pivoter value in degrees. 
    SmartDashboard.putNumber("Pivoter Rotations:", getPivoterRotation()); // Get the pivoter encoder rotation.
    SmartDashboard.putNumber("Pivoter Speed:", getPivoterSpeed()); // Get the speed of the pivoter. 
    SmartDashboard.putBoolean("Pivot Switch:", getSwitch()); // Get the state of the limit switch. 

  }
}


class Gains {
  public final double kP;
	public final double kI;
	public final double kD;
	public final double kF;
	public final double kIzone;
	public final double kPeakOutput;
	
	public Gains(double _kP, double _kI, double _kD, double _kF, double _kIzone, double _kPeakOutput){
		kP = _kP;
		kI = _kI;
		kD = _kD;
		kF = _kF;
		kIzone = _kIzone;
		kPeakOutput = _kPeakOutput;
    }
}
