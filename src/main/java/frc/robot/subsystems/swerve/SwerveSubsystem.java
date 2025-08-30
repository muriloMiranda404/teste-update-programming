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
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.swerve;
import frc.robot.subsystems.LimelightConfig;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase{

  private SwerveDrive swerveDrive;
  private Pigeon2 pigeon;
  private PIDController xPID;
  private PIDController yPID;
  private ProfiledPIDController profilePid;
  private HolonomicDriveController driveController;
  private LimelightConfig limelightConfig;
  private SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  private static Timer timer;

  public static SwerveSubsystem mInstance = null;

  private SwerveSubsystem(File directory){
    try{
      this.swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
        swerveDrive.kinematics, 
        pigeon.getRotation2d(), 
        swerveDrive.getModulePositions(), 
        new Pose2d());

        this.limelightConfig = LimelightConfig.getInstance();
        this.timer = new Timer();

      this.swerveDrive = new SwerveParser(directory).createSwerveDrive(swerve.MAX_SPEED);
    } catch(Exception e){
      System.out.println("erro ao criar o swervedrive");
    }finally{
      xPID = new PIDController(1.0, 0.0, 0.1); // Controle de posição X
      yPID = new PIDController(1.0, 0.0, 0.1); // Controle de posição Y
      profilePid = new ProfiledPIDController(1.0, 0.0, 0.1, new TrapezoidProfile.Constraints(Math.PI, Math.PI)); // Controle de rotação
      
      xPID.setTolerance(0.05); // 5cm de tolerância
      yPID.setTolerance(0.05); // 5cm de tolerância
      profilePid.setTolerance(0.05); // ~3 graus de tolerância
      
      driveController = new HolonomicDriveController(xPID, yPID, profilePid);
      driveController.setEnabled(true); 
      
      pigeon = new Pigeon2(9);

      swerveDrive.setChassisDiscretization(true, 0.2);
    }
  }

  public static SwerveSubsystem getInstance(){
    if(mInstance == null){
      mInstance = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    }
    return mInstance;
  }

  @Override
  public void periodic() {
    swerveDrivePoseEstimator.update(pigeon.getRotation2d(), swerveDrive.getModulePositions());  
    swerveDrive.updateOdometry();

    if(limelightConfig.getHasTarget()){
      Pose2d poseEstimated = limelightConfig.getEstimatedGlobalPose();
      swerveDrivePoseEstimator.addVisionMeasurement(poseEstimated, Timer.getFPGATimestamp());
    }
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

  /**
   * Comando de direção com controle de malha fechada ativado por padrão
   * @param X Fornecedor de velocidade no eixo X
   * @param Y Fornecedor de velocidade no eixo Y
   * @param rotation Fornecedor de velocidade de rotação
   * @return Comando para dirigir o robô
   */
  public Command driveCommand(DoubleSupplier X, DoubleSupplier Y, DoubleSupplier rotation) {
    return driveCommand(X, Y, rotation, true); // Usa malha fechada por padrão
  }
  
  /**
   * Comando de direção com opção de escolher entre malha aberta ou fechada
   * @param X Fornecedor de velocidade no eixo X
   * @param Y Fornecedor de velocidade no eixo Y
   * @param rotation Fornecedor de velocidade de rotação
   * @param useClosedLoop Se true, usa controle de malha fechada; se false, usa malha aberta
   * @return Comando para dirigir o robô
   */
  public Command driveCommand(DoubleSupplier X, DoubleSupplier Y, DoubleSupplier rotation, boolean useClosedLoop) {
    return run(() -> {
      double xController = Math.pow(X.getAsDouble(), 3);
      double yController = Math.pow(Y.getAsDouble(), 3);
      double rotationValue = rotation.getAsDouble();
      Rotation2d rotation2d = swerveDrivePoseEstimator.getEstimatedPosition().getRotation();

      ChassisSpeeds targetSpeeds = swerveDrive.swerveController.getTargetSpeeds(
          xController, 
          yController, 
          rotationValue, 
          getGyroAccum().getRadians(), 
          swerve.MAX_SPEED);
      
      if (useClosedLoop) {
        Pose2d currentPose = getPose();
        
        double dt = 0.02; 
        Pose2d desiredPose = new Pose2d(
            currentPose.getX() + targetSpeeds.vxMetersPerSecond * dt,
            currentPose.getY() + targetSpeeds.vyMetersPerSecond * dt,
            currentPose.getRotation().plus(new Rotation2d(targetSpeeds.omegaRadiansPerSecond * dt)));
        
        ChassisSpeeds adjustedSpeeds = driveController.calculate(
            currentPose, 
            desiredPose,
            targetSpeeds.vxMetersPerSecond,
            rotation2d);
        
        driveFieldOriented(adjustedSpeeds);
      } else {
        driveFieldOriented(targetSpeeds);
      }
    });
  }

  public void driveFieldOriented(ChassisSpeeds speed){
    swerveDrive.driveFieldOriented(speed);
  }

  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(scope0To360(pigeon.getYaw().getValueAsDouble()));
  }

  public Command getAutonomousCommand(String pathName, boolean odom){
    return new PathPlannerAuto(pathName);
  }

  public Rotation2d getGyroAccum(){
    double acumulo = pigeon.getAccumGyroX().getValueAsDouble();

    if(acumulo > 360){
      pigeon.reset();
    }

    acumulo = Math.IEEEremainder(acumulo, 360);

    return Rotation2d.fromDegrees(acumulo);
  }

  public double getYaw(){
    return pigeon.getYaw().getValueAsDouble();
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
  
  /**
   * Configura os ganhos PID para o controlador holonômico de malha fechada
   * @param xKp Ganho proporcional para o controlador X
   * @param xKi Ganho integral para o controlador X
   * @param xKd Ganho derivativo para o controlador X
   * @param yKp Ganho proporcional para o controlador Y
   * @param yKi Ganho integral para o controlador Y
   * @param yKd Ganho derivativo para o controlador Y
   * @param rotKp Ganho proporcional para o controlador de rotação
   * @param rotKi Ganho integral para o controlador de rotação
   * @param rotKd Ganho derivativo para o controlador de rotação
   */
  public void configurePIDGains(double xKp, double xKi, double xKd,
                               double yKp, double yKi, double yKd,
                               double rotKp, double rotKi, double rotKd) {
    // Atualizar ganhos do controlador X
    xPID.setP(xKp);
    xPID.setI(xKi);
    xPID.setD(xKd);
    
    // Atualizar ganhos do controlador Y
    yPID.setP(yKp);
    yPID.setI(yKi);
    yPID.setD(yKd);
    
    // Atualizar ganhos do controlador de rotação
    profilePid.setP(rotKp);
    profilePid.setI(rotKi);
    profilePid.setD(rotKd);
  }

  public double scope0To360(double value){
    value %= 360;

    return value;
  }
  
  /**
   * Verifica se o controlador holonômico atingiu a posição desejada
   * @return true se todos os controladores estiverem dentro da tolerância
   */
  public boolean atReference() {
    return xPID.atSetpoint() && yPID.atSetpoint() && profilePid.atGoal();
  }
}