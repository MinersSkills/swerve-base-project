package frc.robot.Constants;

public class DrivebaseConstants {
    public class tolerance{
        public static final double METER_TOLERANCE = 0.05;
        public static final double ANGLE_TOLERANCE_DEG = 2.0;
    }

    public class TargetX_PID{
        public static final double TARGET_X_P = 1.2;
        public static final double TARGET_X_I = 0.0;
        public static final double TARGET_X_D = 0.0;
    }

    public class TargetY_PID{
        public static final double TARGET_Y_P = 1.2;
        public static final double TARGET_Y_I = 0.0;
        public static final double TARGET_Y_D = 0.0;
    }

    public class TargetRot{
        public static final double TARGET_ROT_P = 0.01;
        public static final double TARGET_ROT_I = 0.0;
        public static final double TARGET_ROT_D = 0.001;  
    }
}
