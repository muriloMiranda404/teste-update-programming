package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveConstants {
    
    public static final double MAX_SPEED = 7.0;
    public static final boolean IS_INVERTED_DRIVE = false;
    public static final boolean IS_INVERTED_ANGLE = false;

    public static final boolean CANCODER_INVERTED = false;
    
    public static final double ROTATION_PER_METER = 0.0472867872006997;
    public static final double TRACK_WIDTH = 0.551942;
    public static final double WHEEL_BASE = 0.551942;

    public static final boolean FIELD_ORIENTED = true;

    public static final double KP_DRIVE = 0.01;
    public static final double KI_DRIVE = 0;
    public static final double KD_DRIVE = 0;

    public static final double KP_ANGLE = 0.01;
    public static final double KI_ANGLE = 0;
    public static final double KD_ANGLE = 0;

    public static final double DRIVE_RAMP = 0.6;
    public static final double ANGLE_RAMP = 0.2;
    
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(0.3556, 0.3556),
      new Translation2d(0.3556, -0.3556),
      new Translation2d(-0.3556, 0.3556),
      new Translation2d(-0.3556, -0.3556)
    );

}
