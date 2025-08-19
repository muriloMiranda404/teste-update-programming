package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LoggedRobot;
import frc.robot.subsystems.controllers.DriverController;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class teste extends LoggedRobot{

    ElevatorSubsystem elevatorSubsystem;
    IntakeSubsystem intakeSubsystem;
    SwerveSubsystem swerveSubsystem;
    LimelightConfig limelightConfig;
    DriverController driverController;

    public teste(){
        this.driverController = DriverController.getInstance();
        this.elevatorSubsystem = ElevatorSubsystem.getInstance();
        this.intakeSubsystem = IntakeSubsystem.getInstance();
        this.swerveSubsystem = SwerveSubsystem.getInstance();
    }

    @AutoLogOutput
    public double getLeftYDrive(){
        return driverController.getLeftY();
    }

    @AutoLogOutput
    public double getLeftXDrive(){
        return driverController.getLeftX();
    }

    @AutoLogOutput
    public double getRightXDrive(){
        return driverController.getRightX();
    }

    @AutoLogOutput
    public double getRightYDrive(){
        return driverController.getRightY();
    }

    @AutoLogOutput
    public double getLeftElevatorOutput(){
        return elevatorSubsystem.getOutputInElevatorMotors()[1];
    }

    @AutoLogOutput
    public double getRightElevatorOutput(){
        return elevatorSubsystem.getOutputInElevatorMotors()[0];
    }
}
