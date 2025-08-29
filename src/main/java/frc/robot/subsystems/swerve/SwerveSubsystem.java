package frc.robot.subsystems.swerve;

import java.io.File;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.swerve;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase{

  SwerveDrive swerveDrive;
  Pigeon2 pigeon;
  PIDController xPID;
  PIDController yPID;
  ProfiledPIDController profilePid;
  HolonomicDriveController driveController;

  public static SwerveSubsystem mInstance = null;

  private SwerveSubsystem(File directory){
    try{
      swerveDrive = new SwerveParser(directory).createSwerveDrive(swerve.MAX_SPEED);
    } catch(Exception e){
      System.out.println("erro ao criar o swervedrive");
    }finally{
      xPID = new PIDController(0.01, 0, 0);
      yPID = new PIDController(0.01, 0, 0);
      profilePid = new ProfiledPIDController(0.01, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));
      driveController = new HolonomicDriveController(yPID, xPID, profilePid);
      pigeon = new Pigeon2(9);
    }
  }

  public static SwerveSubsystem getInstance(){
    if(mInstance == null){
      mInstance = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    }
    return mInstance;
  }

  public void setupPathPlanner(){

    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
      
      boolean feedforwards = true;
      
      AutoBuilder.configure(
            this::getPose,
            this::resetOdometry, 
            this::getRobotRelativeSpeeds, 
            (speeds, feedforward) -> {
              if (feedforwards)
              {
                swerveDrive.drive(
                    speeds,
                    swerveDrive.kinematics.toSwerveModuleStates(speeds),
                    feedforward.linearForces()
                                 );
              } else
              {
                swerveDrive.setChassisSpeeds(speeds);
              }}, 
              new PPLTVController(0.02), 
              config, 
              () -> {
                
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
            );
          } catch (Exception e) {
            e.printStackTrace();
          }
  }

  public Pose2d getPose(){
    return swerveDrive.getPose();
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return swerveDrive.getRobotVelocity();
  }

  public Command driveCommand(DoubleSupplier X, DoubleSupplier Y, DoubleSupplier rotation){
    return run(() ->{
 
      double xController = Math.pow(X.getAsDouble(), 3);
      double yController = Math.pow(Y.getAsDouble(), 3);

      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xController, yController, 
                                                                      rotation.getAsDouble(), 
                                                                      getHeading().getRadians(), 
                                                                      swerve.MAX_SPEED));
    });
  }

  public void driveFieldOriented(ChassisSpeeds speed){
    swerveDrive.driveFieldOriented(speed);
  }

  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
  }

  public Command getAutonomousCommand(String pathName, boolean odom){
    return new PathPlannerAuto(pathName);
  }

  public void setMotorBrake(boolean brake){
    swerveDrive.setMotorIdleMode(brake);
  }

  public void resetOdometry(Pose2d pose){
    swerveDrive.resetOdometry(pose);
  }

  public void drive(Translation2d translation2d, double rotation, boolean fieldOriented){
    swerveDrive.drive(translation2d, rotation, fieldOriented, false);
  }

  public void zeroGyro(){
    swerveDrive.zeroGyro();
  }
}