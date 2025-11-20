package frc.robot.commands.LimeLight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimeLightHelpers;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * A command that aligns the robot to a detected AprilTag using Limelight
 * pose estimation.
 *
 * <p>The command locks onto the currently visible tag at initialization and uses
 * three PID controllers (X, Y, rotation) to move the robot toward the defined
 * alignment setpoints.
 *
 * <p>The command finishes automatically when the robot holds a valid pose
 * for a configured duration or when the tag is not visible for too long.
 *
 * @see frc.robot.LimeLightHelpers
 */
public class AlignToTag extends Command {

    /** The swerve subsystem used to drive the robot. */
    private final SwerveSubsystem swerve;

    /** PID controller for forward/backward alignment (target-space Z). */
    private final PIDController xController;

    /** PID controller for lateral alignment (target-space X). */
    private final PIDController yController;

    /** PID controller for rotational alignment (target-space yaw). */
    private final PIDController rotController;

    /** The fixed ID of the tag detected at initialization. */
    private double tagid = -1;

    /** Timer that measures how long the tag has not been seen. */
    private Timer dontSeeTagTimer;

    /** Timer that measures how long the robot has stayed within PID tolerances. */
    private Timer stopTimer;

    /**
     * Creates a new AlignToTag command.
     *
     * @param swerve the swerve subsystem used to command chassis movement
     */
    public AlignToTag(SwerveSubsystem swerve) {
        this.swerve = swerve;

        xController = new PIDController(
                LimeLightConstants.PidValues.X_ALIGNMENT_P,
                0.0,
                LimeLightConstants.PidValues.X_ALINGMENT_D
        );

        yController = new PIDController(
                LimeLightConstants.PidValues.Y_ALINGMENT_P,
                0.0,
                LimeLightConstants.PidValues.Y_ALINGMENT_D
        );

        rotController = new PIDController(
                LimeLightConstants.PidValues.ROT_ALINGMENT_P,
                0.0,
                LimeLightConstants.PidValues.ROT_ALINGMENT_D
        );

        addRequirements(swerve);
    }

    /**
     * Initializes the command by starting timers, configuring PID setpoints,
     * and locking onto the currently visible AprilTag.
     */
    @Override
    public void initialize() {
        stopTimer = new Timer();
        dontSeeTagTimer = new Timer();
        stopTimer.start();
        dontSeeTagTimer.start();

        xController.setSetpoint(LimeLightConstants.Setpoints.X_SETPOINT_ALINGMENT);
        xController.setTolerance(LimeLightConstants.Tolerance.X_TOLERANCE_ALINGMENT);

        yController.setSetpoint(LimeLightConstants.Setpoints.Y_SETPOINT_ALINGMENT);
        yController.setTolerance(LimeLightConstants.Tolerance.Y_TOLERANCE_ALINGMENT);

        rotController.setSetpoint(LimeLightConstants.Setpoints.ROT_SETPOINT_ALINGMENT);
        rotController.setTolerance(LimeLightConstants.Tolerance.ROT_TOLERANCE_ALINGMENT);

        tagid = LimeLightHelpers.getFiducialID("");
    }

    /**
     * Executes the alignment process. If the locked tag is visible, the robot updates
     * PID outputs based on target-space pose. If the tag is not visible, the robot stops.
     */
    @Override
    public void execute() {
        if (LimeLightHelpers.getTV("") && LimeLightHelpers.getFiducialID("") == tagid) {

            dontSeeTagTimer.reset();

            double[] positions = LimeLightHelpers.getBotPose_TargetSpace("");

            SmartDashboard.putNumber("X_targetspace", positions[2]);
            SmartDashboard.putNumber("Z_targetspace", positions[0]);
            SmartDashboard.putNumber("Yaw_targetspace", positions[4]);

            double xSpeed = xController.calculate(positions[2]);
            double ySpeed = -yController.calculate(positions[0]);
            double rotSpeed = -rotController.calculate(positions[4]);

            swerve.drive(new Translation2d(xSpeed, ySpeed), rotSpeed, false);

            if (!xController.atSetpoint()
                    || !yController.atSetpoint()
                    || !rotController.atSetpoint()) {

                stopTimer.reset();
            }

        } else {
            swerve.drive(new Translation2d(0, 0), 0, false);
        }

        SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
    }

    /**
     * Determines whether the command should finish.
     *
     * @return true if the tag was lost for too long or if the pose was held
     *         stably for the configured validation duration
     */
    @Override
    public boolean isFinished() {
        return dontSeeTagTimer.hasElapsed(LimeLightConstants.Time.DONT_SEE_TAG_WAIT_TIME)
                || stopTimer.hasElapsed(LimeLightConstants.Time.POSE_VALIDATION_TIME);
    }
}
