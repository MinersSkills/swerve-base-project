// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Constants.OperatorConstants;
import frc.robot.commands.LimeLight.AlignToTag;
import frc.robot.commands.swervedrive.drivebase.DriveToPoseCommand;
import frc.robot.dashboard.Dashboards;
import frc.robot.joystick.KeyboardController;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {
        SendableChooser<Command> autoChooser;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    final CommandXboxController driverXbox = new CommandXboxController(0);
    final KeyboardController keyboardController = new KeyboardController();
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve/neo"));

    public Dashboards dashboards = new Dashboards();

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> -driverXbox.getLeftY(),
            () -> -driverXbox.getLeftX())
            .withControllerRotationAxis(driverXbox::getRightX)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(1)
            .allianceRelativeControl(true);

    /**
     * Clone's the angular velocity input stream and converts it to a fieldRelative
     * input stream.
     */
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> driverXbox.getRightX() * -1,
                                                                                               () -> driverXbox.getRightY() * -1)
            .headingWhile(true)
            .driveToPose(() -> getRetreatPose(FieldConstants.Reef.centerFaces[6], -0.7),
                                        new ProfiledPIDController(5, 0, 0, new Constraints(.5, .5)), 
                                        new ProfiledPIDController(1,0, 0, new Constraints(Units.degreesToRadians(90), Units.degreesToRadians(180))));

    /**
     * Clone's the angular velocity input stream and converts it to a robotRelative
     * input stream.
     */
    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true);

    SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> -driverXbox.getLeftY(),
            () -> -driverXbox.getLeftX())
            .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                    2))
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);
    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
            .withControllerHeadingAxis(() -> Math.sin(
                    driverXbox.getRawAxis(
                            2) *
                            Math.PI)
                    *
                    (Math.PI *
                            2),
                    () -> Math.cos(
                            driverXbox.getRawAxis(
                                    2) *
                                    Math.PI)
                            *
                            (Math.PI *
                                    2))
            .headingWhile(true)
            .translationHeadingOffset(true)
            .translationHeadingOffset(Rotation2d.fromDegrees(
                    0));

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
        // Configure the trigger bindingss
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);

        NamedCommands.registerCommand("DriveToPoseReef", Commands.run(() -> {driveDirectAngle.driveToPoseEnabled(true);
        driveFieldOrientedDirectAngle.schedule();}));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary predicate, or via the
     * named factories in
     * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
     * for
     * {@link CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
     * Flight joysticks}.
     */
    private void configureBindings() {

        Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
        drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);

        driverXbox.back().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));

        keyboardController.getATrigger().onTrue(new AlignToTag(drivebase));

        // driverXbox.b().onTrue(new DriveToPoseCommand(drivebase, new Pose2d(2, 0.0, new Rotation2d())));

        driverXbox.rightBumper().onTrue(new AlignToTag(drivebase));

        // driverXbox.y().whileTrue(Commands.run(() -> drivebase.alignToCoralStationRightSide(() -> driverXbox.getLeftX(), () -> driverXbox.getLeftY()), drivebase));

        // driverXbox.x().whileTrue(Commands.run(() -> drivebase.alignToCoralStationLeftSide(() -> driverXbox.getLeftX(), () -> driverXbox.getLeftY()), drivebase));

        Pose2d reef = new Pose2d(13, 4, new Rotation2d(13, 4));

        driveDirectAngle.aim(reef);
        driverXbox.b().whileTrue(Commands.runEnd(() -> driveDirectAngle.aimWhile(true), () -> driveDirectAngle.aimWhile(false)));

        driveDirectAngle.driveToPose(() -> getRetreatPose(FieldConstants.Reef.centerFaces[6], -0.7),
        new ProfiledPIDController(5, 0, 0, new Constraints(.5, .5)), 
        new ProfiledPIDController(1,0, 0, new Constraints(Units.degreesToRadians(90), Units.degreesToRadians(180))));

        driverXbox.y().onTrue(Commands.runEnd(() -> driveDirectAngle.driveToPoseEnabled(true), 
                                                () -> driveDirectAngle.driveToPoseEnabled(false)).until(() -> driveDirectAngle.atTargetPose(0.01)));

        Field2d field = new Field2d();
        field.setRobotPose(getRetreatPose(FieldConstants.Reef.centerFaces[6], -0.7));

        SmartDashboard.putData("Target pose", field);

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public Command alignSwerveWithAngle(double angleDegrees){
        SwerveInputStream alignWithAngles = driveDirectAngle.copy().withControllerHeadingAxis(
        () -> {return Rotation2d.fromDegrees(angleDegrees).getCos();}, 
        () -> {return Rotation2d.fromDegrees(angleDegrees).getSin();});
        return drivebase.driveFieldOriented(alignWithAngles);
    }

    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autoChooser.getSelected();
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }

    public static Pose2d getRetreatPose(Pose2d pose, double distance){
        Transform2d backwardsTransform = new Transform2d(distance, 0, new Rotation2d());
        return pose.plus(backwardsTransform);
    }
}
