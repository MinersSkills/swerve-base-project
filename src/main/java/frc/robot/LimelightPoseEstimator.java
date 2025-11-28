package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightPoseEstimator {

    private Pose2d poseMegaTag2 = new Pose2d();
    private double timestampSeconds = 0.0;
    private boolean doRejectUpdate = false;

    private Field2d field2d = new Field2d();

    public void updateEstimatePose(double yawDegrees, double yawRate){
        LimeLightHelpers.SetRobotOrientation("limelight", yawDegrees, 0, 0, 0, 0, 0);
        LimeLightHelpers.PoseEstimate mt2 = LimeLightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        doRejectUpdate = false;

        int[] ids = {7};

        LimeLightHelpers.SetFiducialIDFiltersOverride("limelight", ids); // enxerga so a 7
         
        // if our angular velocity is greater than 360 degrees per second, ignore vision updates
        if(Math.abs(yawRate) > 360 || mt2.tagCount == 0)
        {
          doRejectUpdate = true;
        }
        if(mt2.tagCount == 0)
        {
          doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
            poseMegaTag2 = mt2.pose;
            timestampSeconds = mt2.timestampSeconds;

            field2d.setRobotPose(mt2.pose);

            SmartDashboard.putData("Limelight pose", field2d);
        }
    }

    public Pose2d getEstimatedPose(){
        return poseMegaTag2;
    }

    public double getTimestampSecondsEstimatedPose(){
        return timestampSeconds;
    }

    public boolean isTheLastEstimatedPoseValid(){
        return !doRejectUpdate;
    }
}
