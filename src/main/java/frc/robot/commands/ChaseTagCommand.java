package frc.robot.commands;

import static frc.robot.subsystems.PoseEstimatorSubsystem.ROBOT_TO_CAMERA;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
// import frc.robot.subsystems.SwerveSubsystem;

public class ChaseTagCommand extends Command {
  
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(8, 8);
  
  private static final int TAG_TO_CHASE = 14;
  private static final Transform3d TAG_TO_GOAL = 
      new Transform3d(
          new Translation3d(1.5, 0.0, 0.0),
          new Rotation3d(0.0, 0.0, Math.PI));

  private final PhotonCamera photonCamera;
  private final DrivetrainSubsystem swerveSubsystem;
  private final Supplier<Pose2d> poseProvider;

  private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

  private PhotonTrackedTarget lastTarget;

  private int count = 0;
      ShuffleboardTab tab = Shuffleboard.getTab("Vision");

  public ChaseTagCommand(
        PhotonCamera photonCamera, 
        Supplier<Pose2d> poseProvider,
        DrivetrainSubsystem swerveSubsystem) {
    this.photonCamera = photonCamera;
    this.poseProvider = poseProvider;
    this.swerveSubsystem = swerveSubsystem;

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    lastTarget = null;
    var robotPose = poseProvider.get();
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());

    tab.add("Chase", count).withPosition(10, 10).withSize(2, 0);
  }

  @Override
  public void execute() {

    count = count + 1;

    System.out.println("#############################################################");

    var robotPose2d = poseProvider.get();
    var robotPose = 
        new Pose3d(
            robotPose2d.getX(),
            robotPose2d.getY(),
            0.0, 
            new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));
    
    var photonRes = photonCamera.getLatestResult();
    if (photonRes.hasTargets()) {
      // Find the tag we want to chase
      var targetOpt = photonRes.getTargets().stream()
          .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
          .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
          .findFirst();
      if (targetOpt.isPresent()) {
        var target = targetOpt.get();
        // This is new target data, so recalculate the goal
        lastTarget = target;
        
        // Transform the robot's pose to find the camera's pose
        var cameraPose = robotPose.transformBy(ROBOT_TO_CAMERA);

        // Trasnform the camera's pose to the target's pose
        var camToTarget = target.getBestCameraToTarget();
        var targetPose = cameraPose.transformBy(camToTarget);
        
        // Transform the tag's pose to set our goal
        var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

        // Drive
        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        omegaController.setGoal(goalPose.getRotation().getRadians());
      }
    }
    
    if (lastTarget == null) {
      // No target has been visible
      // swerveSubsystem.stopModules();
      photonCamera.setLED(VisionLEDMode.kOff);
    } else {
      // Drive to the target
      var xSpeed = xController.calculate(robotPose.getX());
      if (xController.atGoal()) {
        xSpeed = 0;
      }

      var ySpeed = yController.calculate(robotPose.getY());
      if (yController.atGoal()) {
        ySpeed = 0;
      }

      var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
      if (omegaController.atGoal()) {
        omegaSpeed = 0;
      }

      // swerveSubsystem.drive(
      //   ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation()));

      photonCamera.setLED(VisionLEDMode.kBlink);
      lastTarget = null;
    }
  }

  @Override
  public void end(boolean interrupted) {
    // swerveSubsystem.stopModules();
  }

}