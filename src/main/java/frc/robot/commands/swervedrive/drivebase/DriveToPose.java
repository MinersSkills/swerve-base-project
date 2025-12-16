package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generalconstants.DriveToPoseConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class DriveToPose extends Command{
    private final SwerveSubsystem swerve;
    private final Pose2d targetPose;
    private final SwerveInputStream stream;

    private final ProfiledPIDController xPID;
    private final ProfiledPIDController yPID;
    private final ProfiledPIDController thetaPID;

    public DriveToPose(SwerveSubsystem swerve, Pose2d targetPose, SwerveInputStream stream) {
        this.swerve = swerve;
        this.targetPose = targetPose;
        this.stream = stream;

        xPID = new ProfiledPIDController(
            DriveToPoseConstants.TranslationPID.KP,
            DriveToPoseConstants.TranslationPID.KI,
            DriveToPoseConstants.TranslationPID.KD,
            new Constraints(
                DriveToPoseConstants.TranslationPID.MAX_VELOCITY,
                DriveToPoseConstants.TranslationPID.MAX_ACELERATION
            )
        );

        yPID = new ProfiledPIDController(
            DriveToPoseConstants.TranslationPID.KP,
            DriveToPoseConstants.TranslationPID.KI,
            DriveToPoseConstants.TranslationPID.KD,
            new Constraints(
                DriveToPoseConstants.TranslationPID.MAX_VELOCITY,
                DriveToPoseConstants.TranslationPID.MAX_ACELERATION
            )
        );

        thetaPID = new ProfiledPIDController(
            DriveToPoseConstants.RotationPID.KP,
            DriveToPoseConstants.RotationPID.KI,
            DriveToPoseConstants.RotationPID.KD,
            new Constraints(
                DriveToPoseConstants.RotationPID.MAX_VELOCITY,
                DriveToPoseConstants.RotationPID.MAX_ACELERATION
            )
        );

        xPID.setTolerance(DriveToPoseConstants.TranslationPID.TOLERANCE);
        yPID.setTolerance(DriveToPoseConstants.TranslationPID.TOLERANCE);
        thetaPID.setTolerance(DriveToPoseConstants.RotationPID.TOLERANCE);

        thetaPID.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        stream.headingWhile(false);

        Pose2d pose = swerve.getPose();

        xPID.reset(pose.getX());
        yPID.reset(pose.getY());
        thetaPID.reset(pose.getRotation().getRadians());

        xPID.setGoal(targetPose.getX());
        yPID.setGoal(targetPose.getY());
        thetaPID.setGoal(targetPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d pose = swerve.getPose();

        double vx = xPID.calculate(pose.getX());
        double vy = yPID.calculate(pose.getY());
        double omega = thetaPID.calculate(pose.getRotation().getRadians());

        ChassisSpeeds speeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                vx,
                vy,
                omega,
                pose.getRotation()
            );

        swerve.drive(speeds);
    }

    @Override
    public boolean isFinished() {
        return xPID.atGoal()
            && yPID.atGoal()
            && thetaPID.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        stream.headingWhile(true);
        swerve.drive(new ChassisSpeeds());
    }
}