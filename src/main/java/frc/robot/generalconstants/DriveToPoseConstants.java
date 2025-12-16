package frc.robot.generalconstants;

import edu.wpi.first.math.util.Units;

public final class DriveToPoseConstants {
    public final class TranslationPID{
        public static final double KP = 5.0;
        public static final double KI = 0.0;
        public static final double KD = 0.0;

        public static final double TOLERANCE = 0.01;

        public static final int MAX_VELOCITY = 3;
        public static final int MAX_ACELERATION = 4;
    }

    public final class RotationPID{
        public static final double KP = 1.0;
        public static final double KI = 0.0;
        public static final double KD = 0.0;

        public static final double TOLERANCE = Units.degreesToRadians(2);

        public static final double MAX_VELOCITY = Units.degreesToRadians(90);
        public static final double MAX_ACELERATION = Units.degreesToRadians(180);
    }
}
