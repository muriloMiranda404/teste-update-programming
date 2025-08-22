package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Positions;
import frc.robot.commands.ResetPigeon;
import frc.robot.commands.level.SetReefLevel;
import frc.robot.commands.level.elevator.ElevatorPosition;
import frc.robot.commands.level.intake.SetIntakeSpeed;
import frc.robot.commands.swerveUtils.AlingToTarget;
import frc.robot.commands.swerveUtils.TurnRobot;
import frc.robot.subsystems.LimelightConfig;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RegisterNamedCommands {

    SwerveSubsystem swerve;
    ElevatorSubsystem elevator;
    IntakeSubsystem intake;
    LimelightConfig limelightConfig;

    public RegisterNamedCommands(){
        this.swerve = SwerveSubsystem.getInstance();
        this.elevator = ElevatorSubsystem.getInstance();
        this.intake = IntakeSubsystem.getInstance();
        this.limelightConfig = LimelightConfig.getInstance();
    }

    public void configureNamedCommands(){
        this.configurePositionsToAutonomous(elevator, intake);
        this.configureSubsystemsUtils();
        this.configureSwerveAutoCommands(swerve, limelightConfig);
    }

    public void configurePositionsToAutonomous(ElevatorSubsystem elevator, IntakeSubsystem intakeSubsystem){

        NamedCommands.registerCommand("L1", new SetReefLevel(
            Positions.L1_POSITION
        ));

        NamedCommands.registerCommand("L2", new SetReefLevel(
            Positions.L2_POSITION
        ));

        NamedCommands.registerCommand("L3", new SetReefLevel(
            Positions.L3_POSITION
        ));

        NamedCommands.registerCommand("L4", new SetReefLevel(
            Positions.L4_POSITION
        ));

        NamedCommands.registerCommand("PROCESSADOR", new SetReefLevel(
            Positions.PROCESSADOR
        ));

        NamedCommands.registerCommand("ALGAE L2", new SetReefLevel(
            Positions.ALGAE_L2
        ));

        NamedCommands.registerCommand("ALGAE L3", new SetReefLevel(
            Positions.ALGAE_L3
        ));
   
    }

    public void configureSwerveAutoCommands(SwerveSubsystem swerve, LimelightConfig limelightConfig){

        NamedCommands.registerCommand("RESET PIGEON", new InstantCommand(() ->{
            new ResetPigeon(new Pigeon2(9));
        }));

        NamedCommands.registerCommand("ALINHAMENTO", new AlingToTarget(true));

        NamedCommands.registerCommand("TURN ROBOT", new TurnRobot(new Pigeon2(9), 45));
    }

    public void configureSubsystemsUtils(){

        NamedCommands.registerCommand("THROW CORAL", new SetIntakeSpeed(0.8));

        NamedCommands.registerCommand("GET CORAL", new InstantCommand(() ->{
            new SetIntakeSpeed(true);
        }));
    }
}
