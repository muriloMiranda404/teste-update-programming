package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.subsystems.swerve.SwerveModulesConstants;
import frc.robot.subsystems.swerve.SwerveModulesConstants.ModuleId;

public class Constants{

  public static final class Controllers{
    //IDs de controles
    public static final int DRIVE_CONTROLLER = 0;
    public static final int INTAKE_CONTROLLER = 1;

    //deadband dos joystick
    public static final double DEADBAND = 0.1;
  }
  
  public static final class swerve{

    public static final double MAX_SPEED = 7.0;
    public static final boolean IS_INVERTED_DRIVE = false;
    public static final boolean IS_INVERTED_ANGLE = false;

    public static final boolean CANCODER_INVERTED = false;
    
    //configurações do swerve    
    public static final double ROTATION_PER_METER = 0.0472867872006997;
    public static final double TRACK_WIDTH = 0.551942;
    public static final double WHEEL_BASE = 0.551942;

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE/2, -TRACK_WIDTH/2),
      FrontLeft.CHASSI_OFFSET_MOD3,
      BackLeft.CHASSI_OFFSET_MOD2,
      BackRight.CHASSIS_OFFSET_MOD1
    );

    //orientado ao campo
    public static final boolean FIELD_ORIENTED = true;

    //pid drive
    public static final double KP_DRIVE = 0.01;
    public static final double KI_DRIVE = 0;
    public static final double KD_DRIVE = 0;

    //pid angle
    public static final double KP_ANGLE = 0.01;
    public static final double KI_ANGLE = 0;
    public static final double KD_ANGLE = 0;

    public static final double DRIVE_RAMP = 0.6;
    public static final double ANGLE_RAMP = 0.2;
  }

  public static final class Intake{
    public static final int INTAKE_MOTOR = 17;
    public static final int CORAL_MOTOR = 18;

    public static final int INTAKE_ENCODER = 1;

    public static final PIDController INTAKE_PID = new PIDController(0.01, 0, 0);

    public static final int CORAL_SWITCH = 0;

    public static final double INTAKE_TOLERANCE = 4.0;

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

    public static final PIDController ELEVATOR_PID = new PIDController(0.01, 0, 0);

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

  public static final class FrontRight{
    public static final ModuleId MOD4_ID = ModuleId.module4;
    public static final int DRIVE_MOD4 = 1;
    public static final int ANGLE_MOD4 = 2;
    public static final int ENCODER_MOD4 = 13;
    public static final Translation2d CHASSI_OFFSET_MOD4 = new Translation2d(swerve.WHEEL_BASE / 2, -swerve.TRACK_WIDTH/2);
    public static final SwerveModulesConstants MOD4 = new SwerveModulesConstants(MOD4_ID, DRIVE_MOD4, ANGLE_MOD4, 
      ENCODER_MOD4, CHASSI_OFFSET_MOD4);
  }

  public static final class FrontLeft{
    public static final ModuleId MOD3_ID = ModuleId.module3;
    public static final int DRIVE_MOD3 = 8;
    public static final int ANGLE_MOD3 = 7;
    public static final int ENCODER_MOD3 = 12;
    public static final Translation2d CHASSI_OFFSET_MOD3 = new Translation2d(swerve.WHEEL_BASE/2, swerve.TRACK_WIDTH);
    public static final SwerveModulesConstants MOD3 = new SwerveModulesConstants(MOD3_ID, DRIVE_MOD3, ANGLE_MOD3, 
      ENCODER_MOD3, CHASSI_OFFSET_MOD3);
  }

  public static final class BackLeft{
    public static final ModuleId MOD2_ID = ModuleId.module2;
    public static final int DRIVE_MOD2 = 6;
    public static final int ANGLE_MOD2 = 5;
    public static final int ENCODER_MOD2 = 11;
    public static final Translation2d CHASSI_OFFSET_MOD2 = new Translation2d(-swerve.WHEEL_BASE/2, swerve.TRACK_WIDTH/2);
    public static final SwerveModulesConstants MOD2 = new SwerveModulesConstants(MOD2_ID, DRIVE_MOD2, ANGLE_MOD2, 
      ENCODER_MOD2, CHASSI_OFFSET_MOD2);
  }

  public static final class BackRight{
    public static final ModuleId MOD1_ID = ModuleId.module1;
    public static final int DRIVE_MOD1 = 3;
    public static final int ANGLE_MOD1 = 4;
    public static final int ENCODER_MOD1 = 10;
    public static final Translation2d CHASSIS_OFFSET_MOD1 = new Translation2d(-swerve.WHEEL_BASE/2, -swerve.TRACK_WIDTH/2);
    public static final SwerveModulesConstants MOD1 = new SwerveModulesConstants(MOD1_ID, DRIVE_MOD1, ANGLE_MOD1, 
      ENCODER_MOD1, CHASSIS_OFFSET_MOD1);
  }
}