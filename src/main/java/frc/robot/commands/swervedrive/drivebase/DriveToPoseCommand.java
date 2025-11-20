package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * A command that drives the robot toward a specified target {@link Pose2d}
 * using the SwerveSubsystem's internal PID-based DriveToPose logic.
 *
 * <p>The command starts the movement in {@link #initialize()}, continuously
 * checks if the robot has reached the target in {@link #isFinished()}, and
 * stops the drivetrain once completed or interrupted.
 *
 * <h2>Example Usage:</h2>
 *
 * <pre>{@code
 * // Bind to a controller button:
 * driverXbox.b().onTrue(
 *     new DriveToPoseCommand(
 *         drivebase,
 *         new Pose2d(2.0, 0.0, new Rotation2d())
 *     )
 * );
 * }</pre>
 *
 * @see SwerveSubsystem#driveToPosePID(Pose2d)
 * @see SwerveSubsystem#atPose(Pose2d)
 * @see SwerveSubsystem#stop()
 */
public class DriveToPoseCommand extends Command {
  private final SwerveSubsystem swerve;
  private final Pose2d targetPose;

  /**
   * Creates a new DriveToPoseCommand.
   *
   * @param swerve The swerve subsystem used to move the robot.
   * @param targetPose The desired target position and rotation on the field.
   */
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
