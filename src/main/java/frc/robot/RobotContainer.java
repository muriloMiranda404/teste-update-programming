package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

  private final DriverController driverController;
  private final MechanismController mechanismController;

  private final SwerveSubsystem swerve;
  private final LimelightConfig limelightConfig;
  
  private final IntakeSubsystem intake;
  private final ElevatorSubsystem elevator;
  private final SuperStruct struct;

  private final RegisterNamedCommands named;

  private final AutoChooser autoChooser;

  public RobotContainer() {
    this.driverController = DriverController.getInstance();
    this.mechanismController = MechanismController.getInstance();

    this.swerve = SwerveSubsystem.getInstance();
    this.limelightConfig = LimelightConfig.getInstance();

    this.intake = IntakeSubsystem.getInstance();
    this.elevator = ElevatorSubsystem.getInstance();
    this.struct = SuperStruct.getInstance();

    this.named = RegisterNamedCommands.getInstance();
    this.named.configureNamedCommands();

    this.autoChooser = AutoChooser.getInstance();

    swerve.setDefaultCommand(swerve.driveRobot(
      () -> driverController.getLeftY(),
      () -> driverController.getLeftX(),
      () -> driverController.getRightX(),
      true
    ));

    configureDriveBindings();
    configureIntakeBindings();
  }

  private void configureDriveBindings() {
    
    driverController.alingRobotOnReef().whileTrue(new AlingToTarget(true));

    driverController.a().onTrue(new InstantCommand(() ->{
       new ResetPigeon(); 
      }));

    driverController.rightBumper().onTrue(new InstantCommand(() ->{
       new TurnRobot(45);
      }));

    driverController.leftBumper().onTrue(new InstantCommand(() ->{
        new TurnRobot(-45);
      }));

    driverController.emergencyInvert().onTrue(new InstantCommand(() ->{
        driverController.Invert();
      }));
  }
  
  private void configureIntakeBindings(){ 
    
    mechanismController.L1Button().onTrue(new ParallelCommandGroup(
      struct.scorePieceOnLevel(StatesToScore.L1),
      new SetIntakeSpeed(true)
    ));
    mechanismController.L2Button().onTrue(struct.scorePieceOnLevel(StatesToScore.L2));
    mechanismController.L3Button().onTrue(struct.scorePieceOnLevel(StatesToScore.L3));
    mechanismController.L4Button().onTrue(struct.scorePieceOnLevel(StatesToScore.L4));    

    mechanismController.ProcessorButton().onTrue(struct.scorePieceOnLevel(StatesToScore.PROCESSOR));
    mechanismController.L2Algae().onTrue(struct.scorePieceOnLevel(StatesToScore.ALGAE_L2));
    mechanismController.L3Algae().onTrue(struct.scorePieceOnLevel(StatesToScore.ALGAE_L3));
}

  public Command getAutonomousCommand() {
    return swerve.getAutonomousCommand(autoChooser.getPathName(), true);
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }
}
