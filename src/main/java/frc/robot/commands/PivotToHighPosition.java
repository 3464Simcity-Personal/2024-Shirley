// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.PivoterConstants;
import frc.robot.subsystems.PivoterSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.PivoterConstants;

public class PivotToHighPosition extends Command {
  /** Creates a new PivotToHighPosition. */
  private final PivoterSubsystem pivoterSub;
  private final double targetPosition;
  private final int PIVOTER_ANGLE_TOLERANCE = 1;
  private final int TARGET_POSITION_COUNT_THRESHOLD = 10;
  private int targetPositionCount;

  public PivotToHighPosition(PivoterSubsystem pivoterSub, double target) {
    this.pivoterSub = pivoterSub;
    targetPosition = target;
    addRequirements(pivoterSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPositionCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivoterSub.pivot(targetPosition); // setpoint is rotations.
    
    double currPositionDegrees = pivoterSub.getPositionDegrees();
    double targetPositionDegrees = pivoterSub.motorRotationsToDegrees(targetPosition);
    double pivoterPositionError = Math.abs(targetPositionDegrees - currPositionDegrees);
    if (pivoterPositionError < PIVOTER_ANGLE_TOLERANCE){
      targetPositionCount++;
    }

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
    return (pivoterSub.getPivoterRotation() >= targetPosition) ||
            (targetPositionCount > TARGET_POSITION_COUNT_THRESHOLD);
  }
}
