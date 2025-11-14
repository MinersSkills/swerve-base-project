package frc.robot.commands.LimeLight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimeLightHelpers;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AlignToTag extends Command {
    private final SwerveSubsystem swerve;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotController;

    private double tagid = -1;

    private Timer dontSeeTagTimer, stopTimer;

    public AlignToTag(SwerveSubsystem swerve) {
        this.swerve = swerve;

        xController = new PIDController(LimeLightConstants.PidValues.X_ALIGNMENT_P, 0.0, 0.0);
        yController = new PIDController(LimeLightConstants.PidValues.Y_ALINGMENT_P, 0.0, 0.0);
        rotController = new PIDController(LimeLightConstants.PidValues.ROT_ALINGMENT_P, 0.0, 0.0);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        stopTimer = new Timer();
        dontSeeTagTimer = new Timer();
        stopTimer.start();
        dontSeeTagTimer.start();

        // X controllers
        xController.setSetpoint(LimeLightConstants.Setpoints.X_SETPOINT_ALINGMENT);
        xController.setTolerance(LimeLightConstants.Tolerance.X_TOLERANCE_ALINGMENT);

        // Y controllers
        yController.setSetpoint(LimeLightConstants.Setpoints.Y_SETPOINT_ALINGMENT);
        yController.setTolerance(LimeLightConstants.Tolerance.Y_TOLERANCE_ALINGMENT);

        // Rotation controllers
        rotController.setSetpoint(LimeLightConstants.Setpoints.ROT_SETPOINT_ALINGMENT);
        rotController.setTolerance(LimeLightConstants.Tolerance.ROT_TOLERANCE_ALINGMENT);

        // Fix the tag id to not trade the target during the alignment
        tagid = LimeLightHelpers.getFiducialID("");
    }

    @Override
    public void execute() {
        if (LimeLightHelpers.getTV("") && LimeLightHelpers.getFiducialID("") == tagid) {
            dontSeeTagTimer.reset();

            // Receiving the values from LimeLightHelpers class
            double[] positions = LimeLightHelpers.getBotPose_TargetSpace("");

            // Putting the values on dashboard
            SmartDashboard.putNumber("X_targetspace", positions[0]);
            SmartDashboard.putNumber("Z_targetspace", positions[2]);
            SmartDashboard.putNumber("Yaw_targetspace", positions[4]);

            /*
             * target-space
             * positions[2] = z (Front/back)
             * positions[0] = x (lateral)
             * positions[4] = yaw
             */

            // Calculating the speeds
            double xSpeed = xController.calculate(positions[2]);
            double ySpeed = yController.calculate(positions[0]);
            double rotSpeed = rotController.calculate(positions[4]);

            swerve.drive(new Translation2d(xSpeed, ySpeed), rotSpeed, false);

            if (!rotController.atSetpoint()
                    || !xController.atSetpoint()
                    || !yController.atSetpoint()) {

                stopTimer.reset();
            }

        } else {
            // if dont see the tag, stop the swerve
            swerve.drive(new Translation2d(), 0, false);
        }

        // Putting the time to debug
        SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
    }

    @Override
    public boolean isFinished() {
        return dontSeeTagTimer.hasElapsed(LimeLightConstants.Time.DONT_SEE_TAG_WAIT_TIME)
                || stopTimer.hasElapsed(LimeLightConstants.Time.POSE_VALIDATION_TIME);
    }
}
