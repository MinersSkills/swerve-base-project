package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivebaseConstants.TargetX_PID;
import frc.robot.Constants.DrivebaseConstants.TargetY_PID;
import frc.robot.Constants.DrivebaseConstants.tolerance;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * {@code OdmDrivebase} — Comando responsável por mover o robô em uma trajetória
 * até uma posição específica no campo, utilizando controle de posição via odometria.
 * <p>
 * Ele utiliza dois controladores PID (um para X e outro para Y) para ajustar a posição
 * com base na odometria fornecida pelo {@link SwerveSubsystem}.
 * <p>
 * O comando também permite definir dinamicamente a velocidade máxima (topSpeed) 
 * passada na criação da instância, para ajustar o comportamento entre movimentos rápidos e precisos.
 * 
 * <p><b>Uso típico:</b></p>
 * <pre>
 * driverXbox.a().onTrue(
 *     new OdmDrivebase(drivebase, 2.0, 3.0, 0.6)
 * );
 * </pre>
 * 
 * @author Bernardo Fernandes Pereira
 * @see frc.robot.subsystems.swervedrive.SwerveSubsystem
 */
public class OdmDrivebase extends Command {
    private final SwerveSubsystem drivebase;
    private final double targetX;
    private final double targetY;
    private final double topSpeed;

    private final PIDController pidX = new PIDController(TargetX_PID.TARGET_X_P, TargetX_PID.TARGET_X_I,
            TargetX_PID.TARGET_X_D);
    private final PIDController pidY = new PIDController(TargetY_PID.TARGET_Y_P, TargetY_PID.TARGET_Y_I,
            TargetY_PID.TARGET_Y_D);

    /**
     * Cria um novo comando {@code OdmDrivebase}.
     *
     * @param drivebase  O subsistema de swerve drive que o comando controla.
     * @param targetX    Coordenada X (em metros) de destino no campo.
     * @param targetY    Coordenada Y (em metros) de destino no campo.
     * @param topSpeed   Velocidade máxima (em m/s) permitida durante o movimento.
     */
    public OdmDrivebase(SwerveSubsystem drivebase, double targetX, double targetY,
            double topSpeed) {
        this.drivebase = drivebase;
        this.targetX = targetX;
        this.targetY = targetY;
        this.topSpeed = topSpeed;

        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        pidX.reset();
        pidY.reset();
        System.out.println("Movendo para frente");
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivebase.getPose();

        // PID output linear
        double vx = pidX.calculate(currentPose.getX(), targetX);
        double vy = pidY.calculate(currentPose.getY(), targetY);

        // Capping the max output for test with safety
        vx = Math.max(-topSpeed, Math.min(topSpeed, vx));
        vy = Math.max(-topSpeed, Math.min(topSpeed, vy));

        // Create ChassisSpeed based on the field
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                vx,
                vy,
                0,
                drivebase.getPose().getRotation());

        // Sends to drive base
        drivebase.driveFieldOriented(speeds);

        System.out.printf("Pose: X=%.2f Y=%.2f Rot=%.2f°%n",
                currentPose.getX(), currentPose.getY(), currentPose.getRotation().getDegrees());
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = drivebase.getPose();
        double distance = currentPose.getTranslation().getDistance(new Translation2d(targetX, targetY));
        return distance < tolerance.METER_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
    }
}