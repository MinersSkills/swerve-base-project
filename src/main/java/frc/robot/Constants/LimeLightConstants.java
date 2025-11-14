package frc.robot.Constants;

import com.pathplanner.lib.config.PIDConstants;

public class LimeLightConstants {

    public static class PidValues {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
        public static final PIDConstants ANGLE_PID = new PIDConstants(0.009, 0.001, 0);

        public static final double X_ALIGNMENT_P = 3;
        public static final double Y_ALINGMENT_P = 3;
        public static final double ROT_ALINGMENT_P = 0.051;

        public static final double X_ALINGMENT_D = 0.01;
        public static final double Y_ALINGMENT_D = 0;
        public static final double ROT_ALINGMENT_D = 0.0001;
    }

    public static class Setpoints {
        public static final double X_SETPOINT_ALINGMENT = -0.25;  
        public static final double Y_SETPOINT_ALINGMENT = 0;
        public static final double ROT_SETPOINT_ALINGMENT = 0;
    }

    public static class Tolerance {
        public static final double X_TOLERANCE_ALINGMENT = 0.05;
        public static final double Y_TOLERANCE_ALINGMENT = 0.02;
        public static final double ROT_TOLERANCE_ALINGMENT = 0.7;
    }

    public static class Time {
        public static final double DONT_SEE_TAG_WAIT_TIME = 1;
        public static final double POSE_VALIDATION_TIME = 0.5;
    }
}
