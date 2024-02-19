// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.PivoterConstants;
import frc.robot.subsystems.PivoterSubsystem;
import frc.robot.Constants.PivoterConstants;

public class PivotToHighPosition extends Command {
  /** Creates a new PivotToHighPosition. */
  private final PivoterSubsystem pivoterSub;
  private final double targetPosition;
  private final double PIVOTER_ANGLE_TOLERANCE = 0.75;
  double pivoterPositionError;

  public PivotToHighPosition(PivoterSubsystem pivoterSub, double target) {
    this.pivoterSub = pivoterSub;
    targetPosition = target;
    addRequirements(pivoterSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // targetPosition =
    // targetPositionCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetPositionRotations = targetPosition;
    pivoterSub.pivot(targetPositionRotations); // setpoint is rotations.
    double currPositionRotations = pivoterSub.getPivoterRotation();
    pivoterPositionError = Math.abs(targetPositionRotations - currPositionRotations);

    SmartDashboard.putNumber("Pivoter Target Rotation", targetPositionRotations);
    // SmartDashboard.putNumber("Pivoter Target Rotation", Units.rotationsToDegrees(targetPosition));
    SmartDashboard.putNumber("Pivoter Reading", currPositionRotations);
    SmartDashboard.putNumber("Pivoter Error", pivoterPositionError);
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //pivoterSub.addFeedFoward();
    // pivoterSub.re
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(pivoterSub.getPivoterRotation() >= PivoterConstants.kPivoterMaxValue){
      return true;
    }
    return (pivoterPositionError < PIVOTER_ANGLE_TOLERANCE);
  }
}
