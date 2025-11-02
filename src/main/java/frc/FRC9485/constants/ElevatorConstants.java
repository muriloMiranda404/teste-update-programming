package frc.FRC9485.constants;

import com.pathplanner.lib.config.PIDConstants;

public class ElevatorConstants {
    
        public static final int RIGHT_ELEVATOR_MOTOR = 14;
    public static final int LEFT_ELEVATOR_MOTOR = 15;

    public static final int DOWN_SWITCH = 2;
    public static final int UP_SWITCH = 3;

    public static final int ENCODER_ELEV_A = 6;
    public static final int ENCODER_ELEV_B = 8;

    public static final PIDConstants ELEVATOR_CONSTANTS = new PIDConstants(0.01, 0, 0);

    public static final double ACELERATION = 2;
    public static final double VELOCITY = 1;

    public static final double ELEVATOR_TOLERANCE = 30.0;

    public static final class ElevatorPositions{
      public static final double PROCESSOR = 1.0;
      public static final double HOME = 0.0;
      public static final double L2 = 210.0;
      public static final double L3 = 769.0;
      public static final double L4 = 1480.0;

      public static final double ALGAE_L2 = 624.0;
      public static final double ALGAE_L3 = 1107.0;
    }
}

