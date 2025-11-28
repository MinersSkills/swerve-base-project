package frc.robot.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimeLightHelpers;

public class Dashboards {

    public void LimelightDashboard() {
        double[] positions = LimeLightHelpers.getBotPose_TargetSpace("");

        SmartDashboard.putNumber("X_targetspace", positions[2]);
        SmartDashboard.putNumber("Y_targetspace", positions[0]);
        SmartDashboard.putNumber("Yaw_targetspace", positions[4]);
    }
}