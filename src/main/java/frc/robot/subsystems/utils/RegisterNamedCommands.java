package frc.robot.subsystems.utils;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.level.intake.SetIntakeSpeed;
import frc.robot.commands.swerveUtils.AlingToTarget;
import frc.robot.commands.swerveUtils.ResetPigeon;
import frc.robot.commands.swerveUtils.TurnRobot;
import frc.robot.subsystems.LimelightConfig;
import frc.robot.subsystems.Mechanism.SuperStructure;
import frc.robot.subsystems.Mechanism.SuperStructure.StatesToScore;
import frc.robot.subsystems.Mechanism.elevator.ElevatorSubsystem;
import frc.robot.subsystems.Mechanism.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RegisterNamedCommands {

    private static SwerveIO swerve;
    private static ElevatorSubsystem elevator;
    private static IntakeSubsystem intake;
    private static LimelightConfig limelightConfig;
    private static SuperStructure struct;

    public static RegisterNamedCommands mInstance = null;

    private RegisterNamedCommands(){
        RegisterNamedCommands.swerve = SwerveSubsystem.getInstance();
        RegisterNamedCommands.elevator = ElevatorSubsystem.getInstance();
        RegisterNamedCommands.intake = IntakeSubsystem.getInstance();
        RegisterNamedCommands.limelightConfig = LimelightConfig.getInstance();
        RegisterNamedCommands.struct = SuperStructure.getInstance();
    }

    public static RegisterNamedCommands getInstance(){
        if(mInstance == null){
            mInstance = new RegisterNamedCommands();
        }
        return mInstance;
    }

    public static void configureNamedCommands(){
        configurePositionsToAutonomous(struct);
        configureSubsystemsUtils(intake);
        configureSwerveAutoCommands(swerve, limelightConfig, elevator);
    }

    private static void configurePositionsToAutonomous(SuperStructure struct){

        NamedCommands.registerCommand("SCORE ON L1", struct.scorePieceOnLevel(StatesToScore.L1));
        NamedCommands.registerCommand("GO TO HOME POSITION", new ParallelCommandGroup(
            struct.scorePieceOnLevel(StatesToScore.L1),
            new SetIntakeSpeed(true)
        ));
        NamedCommands.registerCommand("SCORE ON L2", struct.scorePieceOnLevel(StatesToScore.L2));
        NamedCommands.registerCommand("SCORE ON L3", struct.scorePieceOnLevel(StatesToScore.L3));
        NamedCommands.registerCommand("SCORE ON L4", struct.scorePieceOnLevel(StatesToScore.L4));

        NamedCommands.registerCommand("SCORE ALGAE ON LEVEL 2", struct.scorePieceOnLevel(StatesToScore.ALGAE_L2));
        NamedCommands.registerCommand("SCORE ALGAE ON LEVEL 3", struct.scorePieceOnLevel(StatesToScore.ALGAE_L3));
        NamedCommands.registerCommand("SCORE ON PROCESSOR", struct.scorePieceOnLevel(StatesToScore.PROCESSOR));
    }

    private static void configureSwerveAutoCommands(SwerveIO swerve, LimelightConfig limelightConfig, ElevatorSubsystem elevatorSubsystem){

        NamedCommands.registerCommand("RESET PIGEON", new ResetPigeon());

        NamedCommands.registerCommand("ALINHAMENTO", new AlingToTarget(true));

        NamedCommands.registerCommand("TURN ROBOT", new TurnRobot(45));

        NamedCommands.registerCommand("RESET ELEVATOR", elevatorSubsystem.resetElevator());

        NamedCommands.registerCommand("RESET ODOMETRY CENTER AUTO", new InstantCommand(() ->{
            swerve.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
        }));
    }

    private static void configureSubsystemsUtils(IntakeSubsystem intakeSubsystem){

        NamedCommands.registerCommand("THROW CORAL", new SetIntakeSpeed(0.8));

        NamedCommands.registerCommand("GET CORAL", new SetIntakeSpeed(true));
    }
}
