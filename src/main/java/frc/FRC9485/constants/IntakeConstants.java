package frc.FRC9485.constants;

import edu.wpi.first.math.controller.PIDController;

public class IntakeConstants{

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