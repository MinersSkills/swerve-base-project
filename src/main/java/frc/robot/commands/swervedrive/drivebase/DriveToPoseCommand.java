package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveToPoseCommand extends Command {
  private final SwerveSubsystem swerve;
  private final Pose2d targetPose;

  public DriveToPoseCommand(SwerveSubsystem swerve, Pose2d targetPose) {
    this.swerve = swerve;
    this.targetPose = targetPose;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    swerve.driveToPosePID(targetPose);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return swerve.atPose(targetPose);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }
}
