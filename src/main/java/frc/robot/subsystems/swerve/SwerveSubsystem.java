package frc.robot.subsystems.swerve;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FRC9485.utils.logger.CustomBooleanLog;
import frc.FRC9485.vision.LimelightHelpers;
import frc.robot.GeralConstants.Components;
import frc.robot.subsystems.vision.LimelightConfig;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import edu.wpi.first.math.trajectory.Trajectory;

public class SwerveSubsystem extends SubsystemBase implements SwerveIO{

  private Pigeon2 pigeon = new Pigeon2(Components.PIGEON);

  private PIDController xPID = new PIDController(0.01, 0, 0);
  private PIDController yPID = new PIDController(0.01, 0, 0);

  private ProfiledPIDController profilePid = new ProfiledPIDController(0.01, 0, 0.1, 
                                                                      new TrapezoidProfile.Constraints(Math.PI, Math.PI));

  private HolonomicDriveController driveController = new HolonomicDriveController(yPID, xPID, profilePid);
  private LimelightConfig limelightConfig;
  
  private SwerveDrive swerveDrive;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(new Translation2d[]{
    new Translation2d(0.356, 0.356),
    new Translation2d(-0.356, 0.356),
    new Translation2d(-0.356, 0.356),
    new Translation2d(-0.356, -0.356)
  });

  private SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  private SwerveModuleState[] state;
  private SwerveModule[] modules;
  private SwerveDriveOdometry odometry;
  private SwerveModulePosition[] position;

  private double direcaoX;
  private double direcaoY;
  private double rotacao;

  private double lastMovingTime;
  private boolean isMoving;
  private IdleMode currentIdleMode;
  private CustomBooleanLog swerveIsMoving = new CustomBooleanLog("swerve/ isMoving");

  private CANcoder[] modulesEncoder = new CANcoder[]{
    new CANcoder(10),
    new CANcoder(11),
    new CANcoder(12),
    new CANcoder(13)
  };

  private swerveInputs inputs = new swerveInputs();

  private Field2d field2d;

  public static SwerveSubsystem mInstance = null;

  List<CANcoder> encoder = new ArrayList<>();

  public record SwerveState(double XInput, double YInput, double rotation, boolean isMoving) {}
  private SwerveState swerveState = new SwerveState(direcaoX, direcaoY, rotacao, isMoving);

  private SwerveSubsystem(File directory){

    try {
      this.swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.MAX_SPEED);

      this.swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(kinematics, 
                                                                   getHeading(),
                                                                  swerveDrive.getModulePositions(),
                                                                  new Pose2d());

    } catch (Exception e) {
      e.printStackTrace();
      DriverStation.reportError("Erro criando SwerveDrive: " + e.getMessage(), false);
      this.swerveDrive = null;
      this.swerveDrivePoseEstimator = null;
    }

    for(int i = 10; i < 14; i++){
      encoder.add(new CANcoder(i));
    }

    this.setupPathPlanner();
    this.configureSwerveUtils();
        
    this.kinematics = new SwerveDriveKinematics(new Translation2d[]{
      new Translation2d(0.0365, 0.356), //Fl
      new Translation2d(-0.356, 0.356), //FR
      new Translation2d(0.356, -0.356), //BL
      new Translation2d(-0.356, -0.356) //BR
    });
    
    this.modules = swerveDrive.getModules();
    
    this.odometry = new SwerveDriveOdometry(kinematics, pigeon.getRotation2d(), swerveDrive.getModulePositions());

  }

  public static SwerveSubsystem getInstance(){
    if(mInstance == null){
      mInstance = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    }
    return mInstance;
  }

  @Override
  public void configureSwerveUtils(){
    this.direcaoX = 0;
    this.direcaoY = 0;
    this.rotacao = 0;
    this.lastMovingTime = 0;
    this.isMoving = false;
    this.currentIdleMode = IdleMode.kBrake;

    this.xPID.setTolerance(0.05);
    this.yPID.setTolerance(0.05);
    this.profilePid.setTolerance(0.05);
    this.driveController.setEnabled(true);
  }

  @Override
  public boolean swerveIsStoped() {
    return modulesEncoder[0].getVelocity().getValueAsDouble() == 0 &&
    modulesEncoder[1].getVelocity().getValueAsDouble() == 0 &&
    modulesEncoder[2].getVelocity().getValueAsDouble() == 0 &&
    modulesEncoder[3].getVelocity().getValueAsDouble() == 0;    
  }  

  @Override
  public SwerveState getState(){
    return swerveState;
  }

  @Override
  public void periodic() {
    if (swerveDrive != null) {
      try {
        SwerveDriveTelemetry.updateData();

        swerveDrive.updateOdometry();
      } catch (Exception e) {
        e.printStackTrace();
        DriverStation.reportError("Erro em updateOdometry() e updateData(): " + e.getMessage(), false);
      }
    } else {
      DriverStation.reportWarning("swerveDrive == null em periodic()", false);
    }
  
    if (swerveDrivePoseEstimator != null && pigeon != null && swerveDrive != null) {
      try {
        swerveDrivePoseEstimator.update(pigeon.getRotation2d(), swerveDrive.getModulePositions());
        if (limelightConfig != null && limelightConfig.getHasTarget()) {
          Pose2d poseEstimated = LimelightHelpers.getBotPose2d("");
          swerveDrivePoseEstimator.addVisionMeasurement(poseEstimated, Timer.getFPGATimestamp());
        }
      } catch (Exception e) {
        e.printStackTrace();
        DriverStation.reportError("Erro ao atualizar poseEstimator: " + e.getMessage(), false);
      }
    }
    
    if(odometry != null && pigeon != null){
      try{
        odometry.update(pigeon.getRotation2d(), position);
      } catch(Exception e){
        e.printStackTrace();
        DriverStation.reportError("erro ao atualizar odometria" + e.getMessage(), false);
      }
    } else {
      DriverStation.reportError("a odometria ou o pigeon estão nulos", false);
    }

    SmartDashboard.putData(field2d);
    this.automaticSwerveMode();

    if(inputs != null){
      this.updateInput(inputs);
    } else{
      DriverStation.reportWarning("inputs do swerve não podem ser atualizados devido o objeto ser nulo", null);
      inputs = null;
    }
  }
  
  @Override
  public boolean swerveIsMoving(){
    boolean isMoving = Math.abs(direcaoX) > 0.05 || Math.abs(direcaoY) > 0.05 || Math.abs(rotacao) > 0.05;
    this.isMoving = isMoving;

    return isMoving;
  }

  public Pigeon2 getPigeon(){
    return this.pigeon;
  }

  @Override
  public double getRoll(){
    return pigeon.getRoll().getValueAsDouble();
  }

  @Override
  public double getPicth(){
    return pigeon.getPitch().getValueAsDouble();
  }

  @Override
  public void automaticSwerveMode(){
    boolean moving = swerveIsMoving();

    if (moving) {
      swerveIsMoving.append(true);
      this.lastMovingTime = Timer.getFPGATimestamp();
      if (this.currentIdleMode != IdleMode.kCoast) {
        if (swerveDrive != null) swerveDrive.setMotorIdleMode(false); 
        this.currentIdleMode = IdleMode.kCoast;
      }
    } else {
      swerveIsMoving.append(false);
      if (Timer.getFPGATimestamp() - lastMovingTime > 1.0) {
        if (this.currentIdleMode != IdleMode.kBrake) {
          if (swerveDrive != null) swerveDrive.setMotorIdleMode(true);
          this.currentIdleMode = IdleMode.kBrake;
        }
      }
    }
  }

  @Override
  public SwerveDrive getSwerveDrive(){
    return swerveDrive;
  }

  @Override
  public SwerveDrivePoseEstimator getPoseEstimator(){
    return this.swerveDrivePoseEstimator;
  }

  @Override
  public void setupPathPlanner(){
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
            
      AutoBuilder.configure(
            this::getPose,
            this::resetOdometry, 
            this::getRobotRelativeSpeeds, 
            (speeds, feedforward) -> driveFieldOriented(speeds), 
              new PPLTVController(0.05),
              config, 
              () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this
            );
          } catch (Exception e) {
            e.printStackTrace();
          }
  }

  @Override
  public Pose2d getPose(){
    return swerveDrivePoseEstimator.getEstimatedPosition();
  }

  @Override
  public ChassisSpeeds getRobotRelativeSpeeds(){
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Comando de direção com controle de malha fechada ativado por padrão
   * @param X Fornecedor de velocidade no eixo X
   * @param Y Fornecedor de velocidade no eixo Y
   * @param rotation Fornecedor de velocidade de rotacao
   * @return Comando para dirigir o robô
   */
  public Command alternDriveCommand(DoubleSupplier X, DoubleSupplier Y, DoubleSupplier rotation) {
    return alternDriveCommand(X, Y, rotation, true); // Usa malha fechada por padrão
  }
  
  /**
   * Comando de direção com opção de escolher entre malha aberta ou fechada
   * @param X Fornecedor de velocidade no eixo X
   * @param Y Fornecedor de velocidade no eixo Y
   * @param rotation Fornecedor de velocidade de rotacao
   * @param useClosedLoop Se true, usa controle de malha fechada; se false, usa malha aberta
   * @return Comando para dirigir o robô
   */
  @Override
  public Command alternDriveCommand(DoubleSupplier X, DoubleSupplier Y, DoubleSupplier rotation, boolean useClosedLoop) {
    return run(() -> {
      double xController = Math.pow(X.getAsDouble(), 3);
      double yController = Math.pow(Y.getAsDouble(), 3);
      double rotationValue = rotation.getAsDouble();
      Rotation2d rotation2d = Rotation2d.fromDegrees(0.0);

      if (swerveDrivePoseEstimator != null) {
        rotation2d = swerveDrivePoseEstimator.getEstimatedPosition().getRotation();
      }
      
      ChassisSpeeds targetSpeeds = swerveDrive.swerveController.getTargetSpeeds(
          xController, 
          yController, 
          rotationValue, 
          getGyroAccum().getRadians(), 
          SwerveConstants.MAX_SPEED);
      
          if (useClosedLoop) {
            Pose2d currentPose = getPose();
            double dt = 0.02;
          
            Pose2d desiredPose = new Pose2d(
                currentPose.getX() + targetSpeeds.vxMetersPerSecond * dt,
                currentPose.getY() + targetSpeeds.vyMetersPerSecond * dt,
                currentPose.getRotation().plus(new Rotation2d(targetSpeeds.omegaRadiansPerSecond * dt)));
          
            Trajectory.State desiredState = new Trajectory.State();
            desiredState.poseMeters = desiredPose;
            desiredState.velocityMetersPerSecond =
                Math.hypot(targetSpeeds.vxMetersPerSecond, targetSpeeds.vyMetersPerSecond);
          
            Rotation2d desiredRotation = desiredPose.getRotation();
          
            ChassisSpeeds adjustedSpeeds = driveController.calculate(currentPose, desiredState, desiredRotation);
          
            driveFieldOriented(adjustedSpeeds);
          } else {
            driveFieldOriented(targetSpeeds);
          }
    });
  }

  /**
   * @param x eixo de x do joystick
   * @param y eixo de y do joystick
   * @param omega eixo de rotaçãp
   * @param fromField drive orientado ao campo
   * @return
  **/
  @Override
  public Command driveRobot(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega, BooleanSupplier fromField){
    return run(() ->{

      this.direcaoX = x.getAsDouble();
      this.direcaoY = y.getAsDouble();
      this.rotacao = omega.getAsDouble();

      double td = 0.02;
      ChassisSpeeds speed = fromField.getAsBoolean() ? ChassisSpeeds.fromFieldRelativeSpeeds(direcaoX,
                                                                                      direcaoY,
                                                                                      rotacao,
                                                                                      pigeon.getRotation2d()) 
                                                                                      : new ChassisSpeeds(
                                                                                      direcaoX, 
                                                                                      direcaoY,
                                                                                      rotacao);

      ChassisSpeeds discretize = ChassisSpeeds.discretize(speed, td);
      state = kinematics.toSwerveModuleStates(discretize);
      SwerveDriveKinematics.desaturateWheelSpeeds(state, SwerveConstants.MAX_SPEED);
      
      for(int i = 0; i < state.length; i++){
        modules[i].setDesiredState(state[i], true, true);
      }
    });
  }

  @Override
  public void setModuleState(SwerveModuleState[] state) {
    for(int i =0; i < state.length; i++){
      modules[i].setDesiredState(state[i], true, true);
    }
  }

  @Override
  public void zeroSwerve() {
    double speed = 0.01;

    SwerveModuleState[] zero = new SwerveModuleState[]{
        new SwerveModuleState(speed, new Rotation2d(0)),
        new SwerveModuleState(speed, new Rotation2d(0)),
        new SwerveModuleState(speed, new Rotation2d(0)),
        new SwerveModuleState(speed, new Rotation2d(0))
      };

      this.setModuleState(zero);
  }

  public Command readSwerveForAuto(){
    return run(() -> {
      this.zeroSwerve();
    });
  }

  @Override
  public void driveFieldOriented(ChassisSpeeds speed){
    swerveDrive.driveFieldOriented(speed);
  }

  @Override
public Rotation2d getHeading(){
  return Rotation2d.fromDegrees(scope0To360(pigeon.getYaw().getValueAsDouble()));
}

/**
 *  @param name
 *  @param altern
   *  @return
   */
  @Override
  public Command getAutonomousCommand(String name, boolean altern){
    if(altern){
      return AutoBuilder.buildAuto(name);
    }
    return new PathPlannerAuto(name);
  }
  
  @Override
  public void stopSwerve(){
    this.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
  }

  public Rotation2d getGyroAccum(){
    double acumulo = pigeon.getAccumGyroX().getValueAsDouble();

    if(acumulo > 360){
      pigeon.reset();
    }

    acumulo = Math.IEEEremainder(acumulo, 360);

    return Rotation2d.fromDegrees(acumulo);
  }

  @Override
  public double getYaw(){
    return pigeon.getYaw().getValueAsDouble();
  }

  @Override
  public void setMotorBrake(boolean brake){
    swerveDrive.setMotorIdleMode(brake);
  }

  //teste no robo real
  @Override
  public void resetOdometry(Pose2d pose){
      try {
          pigeon.setYaw(pose.getRotation().getDegrees());
          swerveDrivePoseEstimator.resetPosition(pose.getRotation(), swerveDrive.getModulePositions(), pose);
      } catch (Exception e){
          DriverStation.reportWarning("Falha ao resetar odometria: " + e.getMessage(), false);
      }
  }

  @Override
  public void drive(Translation2d translation2d, double rotation, boolean fieldOriented){
    swerveDrive.drive(translation2d, rotation, fieldOriented, false);
  }

  @Override
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
   * @param rotKp Ganho proporcional para o controlador de rotacao
   * @param rotKi Ganho integral para o controlador de rotacao
   * @param rotKd Ganho derivativo para o controlador de rotacao
   */
  public void configurePIDGains(double xKp, double xKi, double xKd,
                               double yKp, double yKi, double yKd,
                               double rotKp, double rotKi, double rotKd) {

    xPID.setP(xKp);
    xPID.setI(xKi);
    xPID.setD(xKd);
    
    yPID.setP(yKp);
    yPID.setI(yKi);
    yPID.setD(yKd);
    
    profilePid.setP(rotKp);
    profilePid.setI(rotKi);
    profilePid.setD(rotKd);
  }

  @Override
  public double scope0To360(double value){
    value %= 360;

    return value;
  }
  
  public boolean atReference() {
    return xPID.atSetpoint() && yPID.atSetpoint() && profilePid.atGoal();
  }

  @Override
  public Pose2d getAutoPose() {
      return odometry.getPoseMeters();
  }

  @Override
  public void resetOdometryAuto(Pose2d pose2d) {
    pigeon.setYaw(pose2d.getRotation().getDegrees());
    swerveDrivePoseEstimator.resetPosition(pose2d.getRotation(), swerveDrive.getModulePositions(), pose2d);
  }

  @Override
  public IdleMode getIdleMode() {
      return currentIdleMode;
  }

  @Override
  public void updateInput(swerveInputs input) {
      input.heading = getHeading();
      input.idleMode = getIdleMode();
      input.pose = getPose();
      input.swerveIsMoving = swerveIsMoving();
      input.yaw = getYaw();
      input.pitch = getPicth();
      input.roll = getRoll();
      input.isTeleopered = DriverStation.isTeleopEnabled();
  }
}