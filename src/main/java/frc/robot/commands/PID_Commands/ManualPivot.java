// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PID_Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivoterConstants;
import frc.robot.subsystems.PivoterSubsystem;

public class ManualPivot extends Command {
  // Create
  private boolean isUp;
  private PivoterSubsystem pivoterSub;
  private double currentPivoterReading;
  private double target;
  private final double PIVOTER_ANGLE_TOLERANCE = 0.75;
  double pivoterPositionError;

  

  public ManualPivot(PivoterSubsystem pivoterSub, boolean isUp) {
    this.pivoterSub = pivoterSub;
    this.isUp = isUp;
    addRequirements(pivoterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPivoterReading = pivoterSub.getPivoterRotation();
    if (isUp){
      target = currentPivoterReading + 1.5;
    }else{
      target = currentPivoterReading - 1;
    }
    if ((target >= PivoterConstants.kPivoterMaxValue) || target <= 0){
      System.out.println("ENDING");
      // end command automatically if we're done. 
      end(false);
    }
     
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currPositionRotations = pivoterSub.getPivoterRotation();
    pivoterPositionError = Math.abs(target - currPositionRotations);

    SmartDashboard.putNumber("Manual Pivoter Target Rotation", target);
    // SmartDashboard.putNumber("Pivoter Target Rotation", Units.rotationsToDegrees(targetPosition));
    SmartDashboard.putNumber("Manual Pivoter Reading", currPositionRotations);
    SmartDashboard.putNumber("Manual Pivoter Error", pivoterPositionError);
 
    pivoterSub.pivot(target); // setpoint is rotations.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     if(pivoterSub.getPivoterRotation() >= PivoterConstants.kPivoterMaxValue){
      return true;
    }
    return (pivoterPositionError < PIVOTER_ANGLE_TOLERANCE);
  }
}
