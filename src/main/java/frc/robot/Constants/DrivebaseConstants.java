package frc.robot.Constants;

public class DrivebaseConstants {
    public class Max_speed{
        public static final double MAX_SPEED_MPS = 4.5;
        public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = Math.PI;
    }

    public class tolerance{
        public static final double METER_TOLERANCE = 0.05;
        public static final double ANGLE_TOLERANCE_DEG = 2.0;
    }

    public class TargetX_PID{
        public static final double TARGET_X_P = 1.3;
        public static final double TARGET_X_I = 0.0;
        public static final double TARGET_X_D = 0.01;
    }

    public class TargetY_PID{
        public static final double TARGET_Y_P = 1.3;
        public static final double TARGET_Y_I = 0.0;
        public static final double TARGET_Y_D = 0.01;
    }

    public class TargetRot{
        public static final double TARGET_ROT_P = 0.6;
        public static final double TARGET_ROT_I = 0.0;
        public static final double TARGET_ROT_D = 0.05;  
    }
}
