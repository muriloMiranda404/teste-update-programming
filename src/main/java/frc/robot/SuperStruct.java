package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator.ElevatorPositions;
import frc.robot.Constants.Intake.IntakePositions;
import frc.robot.commands.level.intake.IntakePosition;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class SuperStruct extends SubsystemBase{

    private ElevatorSubsystem elevatorSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private double elevatorInput;
    private double intakeInput;

    public SuperStruct(){
        this.elevatorSubsystem = ElevatorSubsystem.getInstance();
        this.intakeSubsystem = IntakeSubsystem.getInstance();

        this.elevatorInput = 0;
        this.intakeInput = 0;
    }

    @Override
    public void periodic() {
        elevatorSubsystem.setElevatorPosition(elevatorInput);
        intakeSubsystem.setPosition(intakeInput);
    }
    
    public enum StatesToScore{
        L1(1),
        L2(2),
        L3(3),
        L4(4),

        ALGAE_L2(5),
        ALGAE_L3(6),
        PRROCESSOR(7);

        int button;
        StatesToScore(int button){
            this.button = button;
        }
    }

    public void scorePieceOnLevel(StatesToScore state){
        switch (state) {
            case L1:
                intakeInput = IntakePositions.ABERTURA_COMUMM;
                elevatorInput = ElevatorPositions.HOME;
                break;
            
            case L2:
                intakeInput = IntakePositions.PUT_CORAL;
                elevatorInput = ElevatorPositions.L2;
            default:
                break;
        }
    }
}
