// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Controllers;
import frc.robot.Constants.Outros;
import frc.robot.Constants.Positions;
import frc.robot.commands.ResetPigeon;
import frc.robot.commands.TurnRobot;
import frc.robot.commands.level.SetReefLevel;
import frc.robot.commands.swerveUtils.AlingToTarget;
import frc.robot.subsystems.LimelightConfig;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.controllers.DriverController;
import frc.robot.subsystems.controllers.IntakeController;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RobotContainer {

  DriverController DriveJoystick;
  IntakeController IntakeJoystick;

  Pigeon2 pigeon2;

  SwerveSubsystem swerve;
  LimelightConfig limelightConfig;
  
  IntakeSubsystem intake;
  ElevatorSubsystem elevator;

  public RobotContainer() {

    this.DriveJoystick = DriverController.getInstance();
    this.IntakeJoystick = new IntakeController(Controllers.INTAKE_CONTROLLER);

    this.pigeon2 = new Pigeon2(Outros.PIGEON);

    this.swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

    this.limelightConfig = LimelightConfig.getInstance();

    this.intake = IntakeSubsystem.getInstance();
    this.elevator = ElevatorSubsystem.getInstance();

    swerve.setDefaultCommand(swerve.driveCommand(
      () -> MathUtil.applyDeadband(DriveJoystick.ConfigureInputs(true, 1), Controllers.DEADBAND), 
      () -> MathUtil.applyDeadband(DriveJoystick.ConfigureInputs(true, 2), Controllers.DEADBAND), 
      () -> MathUtil.applyDeadband(DriveJoystick.ConfigureInputs(true, 3), Controllers.DEADBAND)));


    configureDriveBindings();
    configureIntakeBindings();

  }

  private void configureDriveBindings() {
    
    IntakeJoystick.AlingRobotOnReef().onTrue(new InstantCommand(() -> {
      new AlingToTarget(limelightConfig, swerve, 0, 0);
    }));

    IntakeJoystick.a().onTrue(new InstantCommand(() ->{
      new ResetPigeon(swerve, pigeon2);
    }));

    IntakeJoystick.rightBumper().onTrue(new InstantCommand(() -> {
      new TurnRobot(swerve, pigeon2, 45);
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

}

  public Command getAutonomousCommand() {
    return swerve.getAutonomousCommand(Outros.AUTO, true);
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }
}
