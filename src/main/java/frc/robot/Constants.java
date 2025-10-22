package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;

public class Constants{

  public static final class vision{
    public static final int[] ALL_TAGS = new int[]{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
  }

  public static final class Controllers{
    //IDs de controles
    public static final int DRIVE_CONTROLLER = 0;
    public static final int INTAKE_CONTROLLER = 1;

    //deadband dos joystick
    public static final double DEADBAND = 0.1;
  }
  
  public static final class Intake{
  
    public record IntakeController(double Kp, double Ki, double Kd, double tolerance) {
      public PIDController getController(){
        PIDController controller = new PIDController(Kp, Ki, Kd);
        controller.setTolerance(tolerance);

        return controller;
      }
    }

    public static final double KP = 0.01;
    public static final double Ki = 0.0;
    public static final double KD = 0.0;
    public static final double INTAKE_TOLERANCE = 4.0;

    public static final IntakeController INTAKE_CONTROLLER = new IntakeController(KP, Ki, KD, INTAKE_TOLERANCE);

    public static final int INTAKE_MOTOR = 17;
    public static final int CORAL_MOTOR = 18;

    public static final int INTAKE_ENCODER = 1;

    public static final PIDController INTAKE_PID = INTAKE_CONTROLLER.getController();

    public static final int CORAL_SWITCH = 0;

    public static final double GET_CORAL_SPEED = 0.3;

    public static final class IntakePositions{
      public static final double CONTROL_BALL = 225.0;
      public static final double ABERTURA_COMUMM = 68.0;
      public static final double DEFAULT_POSITION = 55.0;
      public static final double PUT_CORAL = 72.0;
      public static final double PUT_CORAL_ALTERNATIVE = 72.0;
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

  public static final class Positions{
    public static final double L1_POSITION = 1;
    public static final double L2_POSITION = 2;
    public static final double L3_POSITION = 3;
    public static final double L4_POSITION = 4;
    public static final double PROCESSADOR = 5;
    public static final double ALGAE_L2 = 6;
    public static final double ALGAE_L3 = 7;
  }

  public static final class Components{
    public static final String AUTO = "New Auto";
    public static final String LIMELIGHT = "limelight";
    public static final int PIGEON = 9;
  }
}