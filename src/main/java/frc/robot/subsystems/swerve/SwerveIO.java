package frc.robot.subsystems.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem.SwerveState;
import swervelib.SwerveDrive;

public interface SwerveIO {

    public class swerveInputs {
        public Pose2d pose = new Pose2d();
        public double yaw = 0;
        public IdleMode idleMode = IdleMode.kBrake;
        public Rotation2d heading = new Rotation2d(0);
        public boolean swerveIsMoving = false;
        public SwerveModuleState state = new SwerveModuleState();
        public double pitch = 0;
        public double roll = 0;
        public Boolean isTeleopered = false;
    }
    
    void drive(Translation2d translation, double rotation, boolean fieldOriented);

    Command driveRobot(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation, BooleanSupplier fieldOriented);

    void resetOdometry(Pose2d pose2d);

    void zeroGyro();

    void driveFieldOriented(ChassisSpeeds speeds);

    Command getAutonomousCommand(String path, boolean altern);

    double scope0To360(double value);

    void setMotorBrake(boolean brake);

    void stopSwerve();

    Command alternDriveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation, boolean clossedLoop);

    Pose2d getPose();

    void automaticSwerveMode();

    void setupPathPlanner();

    double getYaw();

    double getRoll();

    double getPicth();

    Rotation2d getHeading();

    boolean swerveIsMoving();

    ChassisSpeeds getRobotRelativeSpeeds();

    SwerveDrive getSwerveDrive();

    SwerveDrivePoseEstimator getPoseEstimator();

    SwerveState getState();

    void configureSwerveUtils();

    boolean swerveIsStoped();

    void setModuleState(SwerveModuleState[] state);

    void zeroSwerve();

    Pose2d getAutoPose();

    void resetOdometryAuto(Pose2d pose2d);

    IdleMode getIdleMode();

    void updateInput(swerveInputs input);
}
