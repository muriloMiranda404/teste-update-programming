package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Positions;
import frc.robot.commands.ResetPigeon;
import frc.robot.commands.level.AutoCommands;
import frc.robot.commands.level.intake.SetIntakeSpeed;
import frc.robot.commands.swerveUtils.AlingToTarget;
import frc.robot.commands.swerveUtils.TurnRobot;
import frc.robot.subsystems.LimelightConfig;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RegisterNamedCommands {

    private SwerveSubsystem swerve;
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private LimelightConfig limelightConfig;
    private SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private Pigeon2 pigeon2;

    public static RegisterNamedCommands mInstance = null;

    private RegisterNamedCommands(){
        this.swerve = SwerveSubsystem.getInstance();
        this.elevator = ElevatorSubsystem.getInstance();
        this.intake = IntakeSubsystem.getInstance();
        this.limelightConfig = LimelightConfig.getInstance();
        this.swerveDrivePoseEstimator = swerve.getPoseEstimator();
        this.pigeon2 = new Pigeon2(9);
    }

    public static RegisterNamedCommands getInstance(){
        if(mInstance == null){
            mInstance = new RegisterNamedCommands();
        }
        return mInstance;
    }

    public void configureNamedCommands(){
        this.configurePositionsToAutonomous(elevator, intake);
        this.configureSubsystemsUtils(intake);
        this.configureSwerveAutoCommands(swerve, limelightConfig, elevator);
    }

    private void configurePositionsToAutonomous(ElevatorSubsystem elevator, IntakeSubsystem intakeSubsystem){

        NamedCommands.registerCommand("L1", new AutoCommands(
            Positions.L1_POSITION
        ));

        NamedCommands.registerCommand("L2", new AutoCommands(
            Positions.L2_POSITION
        ));

        NamedCommands.registerCommand("L3", new AutoCommands(
            Positions.L3_POSITION
        ));

        NamedCommands.registerCommand("L4", new AutoCommands(
            Positions.L4_POSITION
        ));

        NamedCommands.registerCommand("PROCESSADOR", new AutoCommands(
            Positions.PROCESSADOR
        ));

        NamedCommands.registerCommand("ALGAE L2", new AutoCommands(
            Positions.ALGAE_L2
        ));

        NamedCommands.registerCommand("ALGAE L3", new AutoCommands(
            Positions.ALGAE_L3
        ));
   
    }

    private void configureSwerveAutoCommands(SwerveSubsystem swerve, LimelightConfig limelightConfig, ElevatorSubsystem elevatorSubsystem){

        NamedCommands.registerCommand("RESET PIGEON", new InstantCommand(() ->{
            new ResetPigeon(pigeon2);
        }));

        NamedCommands.registerCommand("ALINHAMENTO", new AlingToTarget(true));

        NamedCommands.registerCommand("TURN ROBOT", new TurnRobot(new Pigeon2(9), 45));

        NamedCommands.registerCommand("RESET ELEVATOR", new InstantCommand(() ->{
            elevatorSubsystem.resetElevator();
        }));

        NamedCommands.registerCommand("RESET ODOMETRY CENTER AUTO", new InstantCommand(() ->{
            swerve.resetOdometry(new Pose2d(7.492, 3.839, Rotation2d.fromDegrees(179.832)));
        }));

        NamedCommands.registerCommand("RESET POSE FOR CENTER AUTONOMOUS", new InstantCommand(() ->{
            swerveDrivePoseEstimator.resetPosition(
                pigeon2.getRotation2d(),
                swerve.getSwerveDrive().getModulePositions(), 
                new Pose2d(7.492, 3.839, Rotation2d.fromDegrees(179.832)));
        }));
    }

    private void configureSubsystemsUtils(IntakeSubsystem intakeSubsystem){

        NamedCommands.registerCommand("THROW CORAL", new SetIntakeSpeed(0.8));

        NamedCommands.registerCommand("GET CORAL", new InstantCommand(() ->{
            new SetIntakeSpeed(true);
        }));
    }
}
