package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator.ElevatorPositions;
import frc.robot.Constants.Intake.IntakePositions;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class SuperStruct extends SubsystemBase{

    private ElevatorSubsystem elevatorSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private double elevatorInput;
    private double intakeInput;

    private boolean activateGetCoral;

    public SuperStruct(){
        this.elevatorSubsystem = ElevatorSubsystem.getInstance();
        this.intakeSubsystem = IntakeSubsystem.getInstance();
        this.activateGetCoral = false;
        this.elevatorInput = ElevatorPositions.HOME;
        this.intakeInput = IntakePositions.DEFAULT_POSITION;
    }

    @Override
    public void periodic() {
        this.elevatorSubsystem.setElevatorPosition(elevatorInput);
        this.intakeSubsystem.setPosition(intakeInput);

        this.elevatorSubsystem.periodic();
        this.intakeSubsystem.periodic();
    }

    public int getButtonPessed(){
        return getButtonPessed();
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

        public int getButtonPessed(){
            return button;
        }
    }

    public enum ScoreSequence{
        ABRIR_INTAKE,
        SUBIR_ELEVADOR,
        FECHAR_INTAKE
    }

    public void sequenceToScoreL1(){
        ScoreSequence sequence = ScoreSequence.ABRIR_INTAKE;

        switch (sequence) {
            case ABRIR_INTAKE:
                this.intakeInput = IntakePositions.ABERTURA_COMUMM;
                if(this.intakeSubsystem.atSetpoint()) sequence = ScoreSequence.SUBIR_ELEVADOR;
                break;
        
            case SUBIR_ELEVADOR:
                this.elevatorInput = ElevatorPositions.HOME;
                if(this.elevatorSubsystem.atSetpoint()) sequence = ScoreSequence.FECHAR_INTAKE;
                break;

            case FECHAR_INTAKE:
                this.intakeInput = IntakePositions.DEFAULT_POSITION;
                if(this.intakeSubsystem.atSetpoint()) this.activateGetCoralMotor();
                break;
        }
    }

    public void sequenceToScoreL4(){
        ScoreSequence sequence = ScoreSequence.ABRIR_INTAKE;

        switch (sequence) {
            case ABRIR_INTAKE:
                this.intakeInput = IntakePositions.ABERTURA_COMUMM;
                if(this.intakeSubsystem.atSetpoint()) sequence = ScoreSequence.SUBIR_ELEVADOR;
                break;
        
            case SUBIR_ELEVADOR:
                this.elevatorInput = ElevatorPositions.L4;
                if(this.elevatorSubsystem.atSetpoint()) sequence = ScoreSequence.FECHAR_INTAKE;
                break;

            case FECHAR_INTAKE:
                this.intakeInput = IntakePositions.OPEN_L4;
                break;
        }
    }

    public void scorePieceOnLevel(StatesToScore state){
        switch (state) {
            case L1:
               this.sequenceToScoreL1();
                break;
            
            case L2:
                this.intakeInput = IntakePositions.PUT_CORAL;
                if(this.intakeSubsystem.atSetpoint()){
                    this.elevatorInput = ElevatorPositions.L2;
                }    
                break;

            case L3:
                this.intakeInput = IntakePositions.PUT_CORAL;
                if(this.intakeSubsystem.atSetpoint()){
                    this.elevatorInput = ElevatorPositions.L3;
                }    
                break;

            case L4:
                this.sequenceToScoreL4();
                break;

            case ALGAE_L2:
                intakeInput = IntakePositions.CONTROL_BALL;
                elevatorInput = ElevatorPositions.ALGAE_L2;
                break;

            case ALGAE_L3:
                intakeInput = IntakePositions.CONTROL_BALL;
                elevatorInput = ElevatorPositions.ALGAE_L3;
                break;

            case PRROCESSOR:
                intakeInput = IntakePositions.CONTROL_BALL;
                elevatorInput = ElevatorPositions.HOME;
                break;
        }
    }

    public void activateGetCoralMotor(){
        if(activateGetCoral){
            intakeSubsystem.set(0.2);
        }
        if(intakeSubsystem.IsTouched()){
            intakeSubsystem.set(0);
            activateGetCoral = false;
        }
    }

    public void seguranceSetpoint(){
        if(intakeSubsystem.getDistance() < 55){
            elevatorSubsystem.setElevatorSpeed(0);
        }
    }
}
