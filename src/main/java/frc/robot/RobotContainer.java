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
import frc.robot.subsystems.Mechanism.SuperStructure;
import frc.robot.subsystems.Mechanism.SuperStructure.StatesToScore;
import frc.robot.subsystems.Mechanism.elevator.ElevatorSubsystem;
import frc.robot.subsystems.Mechanism.intake.IntakeSubsystem;
import frc.robot.subsystems.controllers.DriverController;
import frc.robot.subsystems.controllers.MechanismJoystick;
import frc.robot.subsystems.controllers.MechanismKeyBoard;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RobotContainer {

  private final DriverController driverController;
  private final MechanismJoystick mechanismController;
  private final MechanismKeyBoard mechanismKeyboard;

  private final SwerveSubsystem swerve;
  
  private final SuperStructure superStructure;

  private final AutoChooser autoChooser;

  public RobotContainer() {

    //joystick
    this.driverController = DriverController.getInstance();
    this.mechanismController = MechanismJoystick.getInstance();
    this.mechanismKeyboard = MechanismKeyBoard.getInstance();

    this.swerve = SwerveSubsystem.getInstance();

    this.superStructure = SuperStructure.getInstance();

    this.autoChooser = AutoChooser.getInstance();

    swerve.setDefaultCommand(swerve.driveRobot(
      () -> driverController.getLeftY(),
      () -> driverController.getLeftX(),
      () -> driverController.getRightX(),
      frc.robot.Constants.swerve.FIELD_ORIENTED
    ));

    configureDriveBindings();
    configureJoystickMechanismBindings();
    configureKeyBoardMechanismBindings();
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

  private void configureKeyBoardMechanismBindings(){

    mechanismKeyboard.L1Button().onTrue(new ParallelCommandGroup(
      superStructure.scorePieceOnLevel(StatesToScore.L1),
      new SetIntakeSpeed(true)
    ));
    mechanismKeyboard.L2Button().onTrue(superStructure.scorePieceOnLevel(StatesToScore.L2));    
    mechanismKeyboard.L3Button().onTrue(superStructure.scorePieceOnLevel(StatesToScore.L3));
    mechanismKeyboard.L4Button().onTrue(superStructure.scorePieceOnLevel(StatesToScore.L4));
    
    mechanismKeyboard.algae_L2().onTrue(superStructure.scorePieceOnLevel(StatesToScore.ALGAE_L2));
    mechanismKeyboard.algae_L3().onTrue(superStructure.scorePieceOnLevel(StatesToScore.ALGAE_L3));
    mechanismKeyboard.Processador().onTrue(superStructure.scorePieceOnLevel(StatesToScore.PROCESSOR));

    mechanismController.throwCoralOnIntake().whileTrue(new SetIntakeSpeed(0.8));
    mechanismKeyboard.getAlgae().whileTrue(new SetIntakeSpeed(-0.1));
  }
  
  private void configureJoystickMechanismBindings(){ 
    
    mechanismController.L1Button().onTrue(new ParallelCommandGroup(
      superStructure.scorePieceOnLevel(StatesToScore.L1),
      new SetIntakeSpeed(true)
    ));
    mechanismController.L2Button().onTrue(superStructure.scorePieceOnLevel(StatesToScore.L2));
    mechanismController.L3Button().onTrue(superStructure.scorePieceOnLevel(StatesToScore.L3));
    mechanismController.L4Button().onTrue(superStructure.scorePieceOnLevel(StatesToScore.L4));    

    mechanismController.ProcessorButton().onTrue(superStructure.scorePieceOnLevel(StatesToScore.PROCESSOR));
    mechanismController.L2Algae().onTrue(superStructure.scorePieceOnLevel(StatesToScore.ALGAE_L2));
    mechanismController.L3Algae().onTrue(superStructure.scorePieceOnLevel(StatesToScore.ALGAE_L3));
  }

  public Command getAutonomousCommand() {
    return swerve.getAutonomousCommand(autoChooser.getPathName(), true);
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }
}
