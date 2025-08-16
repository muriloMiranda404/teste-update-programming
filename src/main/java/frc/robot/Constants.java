package frc.robot;

import edu.wpi.first.math.controller.PIDController;

public class Constants{

  public static final class Controllers{
    public static final int DRIVE_CONTROLLER = 0;
    public static final int INTAKE_CONTROLLER = 1;

    public static final double DEADBAND = 0.1;
  }

  public static final class swerve{
    public static final double MAX_SPEED = 7.0;
  }

  public static final class Intake{
    public static final int INTAKE_MOTOR = 17;
    public static final int CORAL_MOTOR = 18;

    public static final int INTAKE_ENCODER = 1;

    public static final PIDController INTAKE_PID = new PIDController(0.01, 0, 0);

    public static final int CORAL_SWITCH = 0;

    public static final double INTAKE_TOLERANCE = 4.0;

    public static final class IntakePositions{
      public static final double CONTROL_BALL = 225.0;
      public static final double ABERTURA_COMUMM = 68.0;
      public static final double DEFAULT_POSITION = 55.0;
      public static final double PUT_CORAL = 72.0;
      public static final double OPEN_L4 = 92.0;
    }
  }

  public static final class Elevator{
    public static final int RIGHT_ELEVATOR_MOTOR = 14;
    public static final int LEFT_ELEVATOR_MOTOR = 15;

    public static final int DOWN_SWITCH = 2;
    public static final int UP_SWITCH = 3;

    public static final int ENCODER_ELEV_A = 6;
    public static final int ENCODER_ELEV_B = 8;

    public static final PIDController ELEVATOR_PID = new PIDController(0.01, 0, 0);

    public static final double ELEVATOR_TOLERANCE = 30.0;

    public static final class ElevatorPositions{
      public static final double HOME = 0.0;
      public static final double L2 = 210.0;
      public static final double L3 = 769.0;
      public static final double L4 = 1480.0;

      public static final double ALGAE_L2 = 624.0;
      public static final double ALGAE_L3 = 1107.0;
    }
  }

  public static final class Outros{
    public static final String AUTO = "New Auto";
    public static final String LIMELIGHT = "limelight";
    public static final int PIGEON = 9;
  }
}