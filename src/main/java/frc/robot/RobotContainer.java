package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Outros;
import frc.robot.Constants.Positions;
import frc.robot.commands.ResetPigeon;
import frc.robot.commands.level.SetReefLevel;
import frc.robot.commands.swerveUtils.AlingToTarget;
import frc.robot.commands.swerveUtils.TurnRobot;
import frc.robot.subsystems.LimelightConfig;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.controllers.DriverController;
import frc.robot.subsystems.controllers.IntakeController;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RobotContainer {

  DriverController driverJoystick;
  IntakeController IntakeJoystick;

  Pigeon2 pigeon2;

  SwerveSubsystem swerve;
  LimelightConfig limelightConfig;
  
  IntakeSubsystem intake;
  ElevatorSubsystem elevator;

  public RobotContainer() {

    this.driverJoystick = DriverController.getInstance();
    this.IntakeJoystick = IntakeController.getInstance();

    this.pigeon2 = new Pigeon2(Outros.PIGEON);

    this.swerve = SwerveSubsystem.getInstance();

    this.limelightConfig = LimelightConfig.getInstance();

    this.intake = IntakeSubsystem.getInstance();
    this.elevator = ElevatorSubsystem.getInstance();

    swerve.setDefaultCommand(driverJoystick.driverRobot());

    configureDriveBindings();
    configureIntakeBindings();

  }

  private void configureDriveBindings() {
    
    driverJoystick.alingRobotOnReef().onTrue(new InstantCommand(() -> {
      new AlingToTarget( 0, 0);
    }));

    driverJoystick.a().onTrue(new InstantCommand(() ->{
      new ResetPigeon(pigeon2);
    }));

    driverJoystick.rightBumper().onTrue(new InstantCommand(() -> {
      new TurnRobot(pigeon2, 45);
    }));
    
  }
  
  private void configureIntakeBindings(){ 

    IntakeJoystick.L1Button().onTrue( new SetReefLevel(
      Positions.L1_POSITION
    ));

    IntakeJoystick.L2Button().onTrue(new SetReefLevel(
      Positions.L2_POSITION
    ));

    IntakeJoystick.L3Button().onTrue(new SetReefLevel(
      Positions.L3_POSITION
    ));

    IntakeJoystick.L4Button().onTrue(new SetReefLevel(
      Positions.L4_POSITION
    ));

    IntakeJoystick.ProcessorButton().onTrue(new SetReefLevel(
      Positions.PROCESSADOR
    ));

    IntakeJoystick.L2Algae().onTrue(new SetReefLevel(
      Positions.ALGAE_L2
    ));

    IntakeJoystick.L3Algae().onTrue(new SetReefLevel(
      Positions.ALGAE_L3
    ));
}

  public Command getAutonomousCommand() {
    return swerve.getAutonomousCommand(Outros.AUTO, true);
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }
}
