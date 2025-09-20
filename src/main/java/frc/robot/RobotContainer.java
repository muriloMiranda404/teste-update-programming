package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.Components;
import frc.robot.commands.AutoChooser;
import frc.robot.commands.ResetPigeon;
import frc.robot.commands.level.intake.SetIntakeSpeed;
import frc.robot.commands.swerveUtils.AlingToTarget;
import frc.robot.commands.swerveUtils.TurnRobot;
import frc.robot.subsystems.LimelightConfig;
import frc.robot.subsystems.Mechanism.SuperStruct;
import frc.robot.subsystems.Mechanism.SuperStruct.StatesToScore;
import frc.robot.subsystems.Mechanism.elevator.ElevatorSubsystem;
import frc.robot.subsystems.Mechanism.intake.IntakeSubsystem;
import frc.robot.subsystems.controllers.DriverController;
import frc.robot.subsystems.controllers.MechanismController;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.utils.RegisterNamedCommands;

public class RobotContainer {

  private DriverController driverJoystick;
  private MechanismController mechanismJoystick;

  private Pigeon2 pigeon2;

  private SwerveSubsystem swerve;
  private LimelightConfig limelightConfig;
  
  private IntakeSubsystem intake;
  private ElevatorSubsystem elevator;
  private SuperStruct struct;

  private RegisterNamedCommands named;

  private AutoChooser autoChooser;

  public RobotContainer() {
    this.driverJoystick = DriverController.getInstance();
    this.mechanismJoystick = MechanismController.getInstance();

    this.pigeon2 = new Pigeon2(Components.PIGEON);

    this.swerve = SwerveSubsystem.getInstance();
    this.limelightConfig = LimelightConfig.getInstance();

    this.intake = IntakeSubsystem.getInstance();
    this.elevator = ElevatorSubsystem.getInstance();
    this.struct = SuperStruct.getInstance();

    this.named = RegisterNamedCommands.getInstance();
    this.named.configureNamedCommands();

    this.autoChooser = AutoChooser.getInstance();

    swerve.setDefaultCommand(swerve.driveCommand(
      () -> driverJoystick.getLeftY(),
      () -> driverJoystick.getLeftX(),
      () -> driverJoystick.getRightX()
    ));

    configureDriveBindings();
    configureIntakeBindings();
  }

  private void configureDriveBindings() {
    
    driverJoystick.alingRobotOnReef().whileTrue(new AlingToTarget(true));

    driverJoystick.a().onTrue(new InstantCommand(() ->{
       new ResetPigeon(); 
      }));

    driverJoystick.rightBumper().onTrue(new InstantCommand(() ->{
       new TurnRobot(45);
      }));

    driverJoystick.leftBumper().onTrue(new InstantCommand(() ->{
        new TurnRobot(-45);
      }));

    driverJoystick.emergencyInvert().onTrue(new InstantCommand(() ->{
        driverJoystick.Invert();
      }));
  }
  
  private void configureIntakeBindings(){ 
    
    mechanismJoystick.L1Button().onTrue(new ParallelCommandGroup(
      struct.scorePieceOnLevel(StatesToScore.L1),
      new SetIntakeSpeed(true)
    ));
    mechanismJoystick.L2Button().onTrue(struct.scorePieceOnLevel(StatesToScore.L2));
    mechanismJoystick.L3Button().onTrue(struct.scorePieceOnLevel(StatesToScore.L3));
    mechanismJoystick.L4Button().onTrue(struct.scorePieceOnLevel(StatesToScore.L4));    

    mechanismJoystick.ProcessorButton().onTrue(struct.scorePieceOnLevel(StatesToScore.PROCESSOR));
    mechanismJoystick.L2Algae().onTrue(struct.scorePieceOnLevel(StatesToScore.ALGAE_L2));
    mechanismJoystick.L3Algae().onTrue(struct.scorePieceOnLevel(StatesToScore.ALGAE_L3));
}

  public Command getAutonomousCommand() {
    return swerve.getAutonomousCommand(autoChooser.getPathName(), true);
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }
}
