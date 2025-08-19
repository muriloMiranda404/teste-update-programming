package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LoggedRobot;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.subsystems.controllers.DriverController;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class teste extends LoggedRobot{

    ElevatorSubsystem elevatorSubsystem;
    IntakeSubsystem intakeSubsystem;
    SwerveSubsystem swerveSubsystem;
    LimelightConfig limelightConfig;
    DriverController driverController;

    CANcoder FrontRight;
    CANcoder FrontLeft;
    CANcoder BackRight;
    CANcoder BackLeft;

    Pigeon2 pigeon;

    public teste(){
        this.driverController = DriverController.getInstance();
        this.elevatorSubsystem = ElevatorSubsystem.getInstance();
        this.intakeSubsystem = IntakeSubsystem.getInstance();
        this.swerveSubsystem = SwerveSubsystem.getInstance();
        this.limelightConfig = LimelightConfig.getInstance();

        FrontLeft = new CANcoder(10);
        FrontRight = new CANcoder(11);
        BackLeft = new CANcoder(13);
        BackRight = new CANcoder(12);

        pigeon = new Pigeon2(9);
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

    @AutoLogOutput
    public double getFrontLeft(){
        return FrontLeft.getPosition().getValueAsDouble();
    }

    @AutoLogOutput
    public double getFrontRight(){
        return FrontRight.getPosition().getValueAsDouble();
    }

    @AutoLogOutput
    public double getBackLeft(){
        return BackLeft.getPosition().getValueAsDouble();
    }
    
    @AutoLogOutput
    public double getBackRight(){
        return BackRight.getPosition().getValueAsDouble();
    }

    @AutoLogOutput
    public double getGyroscopyYaw(){
        return pigeon.getYaw().getValueAsDouble();
    }

    @AutoLogOutput
    public double getTxSetpoint(){
        return limelightConfig.getAprilTagCordenates()[0];
    }

    @AutoLogOutput
    public double getTySetpoint(){
        return limelightConfig.getAprilTagCordenates()[1];
    }
}
