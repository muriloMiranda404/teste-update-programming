package frc.robot.subsystems.Mechanism;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator.ElevatorPositions;
import frc.robot.Constants.Intake.IntakePositions;
import frc.robot.subsystems.Led.LedSubsystem;
import frc.robot.subsystems.Mechanism.elevator.ElevatorSubsystem;
import frc.robot.subsystems.Mechanism.intake.IntakeSubsystem;

public class SuperStruct extends SubsystemBase{

    private ElevatorSubsystem elevatorSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private LedSubsystem ledSubsystem;

    private double elevatorInput;
    private double intakeInput;

    private String state;
    private Color color;

    public static SuperStruct mInstance = null;

    private SuperStruct(){
        this.elevatorSubsystem = ElevatorSubsystem.getInstance();
        this.intakeSubsystem = IntakeSubsystem.getInstance();
        this.ledSubsystem = LedSubsystem.getInstance();
        this.elevatorInput = ElevatorPositions.HOME;
        this.intakeInput = IntakePositions.DEFAULT_POSITION;
        this.color = Color.kBlack;
        this.state = "INITIAL STATE";
    }

    public static SuperStruct getInstance(){
        if(mInstance == null){
            mInstance = new SuperStruct();
        }
        return mInstance;
    }

    public String getState(){
        return state;
    }

    @Override
    public void periodic() {
        this.elevatorSubsystem.setPosition(elevatorInput);
        this.intakeSubsystem.setPosition(intakeInput);
        alternLedColor(color);
        this.elevatorSubsystem.periodic();
        this.intakeSubsystem.periodic();
    }
    
    public enum StatesToScore{
        L1(1),
        L2(2),
        L3(3),
        L4(4),

        ALGAE_L2(5),
        ALGAE_L3(6),
        PROCESSOR(7);

        int button;
        StatesToScore(int button){
            this.button = button;
        }

        public int getButtonPessed(){
            return button;
        }
    }

    private void alternLedColor(Color color){
        if(intakeSubsystem.IsTouched()){
            ledSubsystem.setPattern(LEDPattern.gradient(GradientType.kDiscontinuous, Color.kRed, color));
        } else if(elevatorSubsystem.getOutputInElevatorMotors()[0] != 0){
            ledSubsystem.setPattern(LEDPattern.gradient(GradientType.kDiscontinuous, Color.kWhite, color));
        } else {
            ledSubsystem.setColor(color);
        }
    }

    public Command scorePieceOnLevel(StatesToScore state){
        return run(() ->{
            switch (state) {
                case L1:
                this.state = "L1 STATE";
                color = Color.kGreen;
                if(this.elevatorInput != ElevatorPositions.HOME) this.intakeInput = IntakePositions.ABERTURA_COMUMM;
                if(this.intakeSubsystem.atSetpoint()){
                    this.elevatorInput = ElevatorPositions.HOME;
                    if(this.elevatorSubsystem.atSetpoint() && intakeSubsystem.getSetpoint() == IntakePositions.ABERTURA_COMUMM){
                        this.intakeInput = IntakePositions.DEFAULT_POSITION;
                    }
                }
                break;
                
                case L2:
                    this.state = "L2 STATE";
                    color = Color.kYellow;
                    this.ledSubsystem.setColor(edu.wpi.first.wpilibj.util.Color.kGray);
                    this.intakeInput = IntakePositions.PUT_CORAL_ALTERNATIVE;
                    if(this.intakeSubsystem.atSetpoint()){
                        this.elevatorInput = ElevatorPositions.L2;
                    }    
                    break;
    
                case L3:
                    this.state = "L3 STATE";
                    color = Color.kPurple;
                    this.intakeInput = IntakePositions.PUT_CORAL_ALTERNATIVE;
                    if(this.intakeSubsystem.atSetpoint()){
                        this.elevatorInput = ElevatorPositions.L3;
                    }    
                    break;
    
                case L4:
                    this.state = "L4 STATE";
                    color = Color.kDarkBlue;
                    if(this.elevatorInput != ElevatorPositions.L4) this.intakeInput = IntakePositions.PUT_CORAL;
                    if(this.intakeSubsystem.atSetpoint()){
                        this.elevatorInput = ElevatorPositions.L4;
                        if(this.elevatorSubsystem.atSetpoint() && this.intakeSubsystem.getSetpoint() == IntakePositions.PUT_CORAL){
                            this.intakeInput = IntakePositions.OPEN_L4;
                        }
                    }
                    break;
    
                case ALGAE_L2:
                    this.state = "L2 ALGAE";
                    intakeInput = IntakePositions.CONTROL_BALL;
                    elevatorInput = ElevatorPositions.ALGAE_L2;
                    break;
    
                case ALGAE_L3:
                    this.state = "L3 ALGAE";
                    intakeInput = IntakePositions.CONTROL_BALL;
                    elevatorInput = ElevatorPositions.ALGAE_L3;
                    break;
    
                case PROCESSOR:
                    this.state = "PROCESSADOR";
                    intakeInput = IntakePositions.CONTROL_BALL;
                    elevatorInput = ElevatorPositions.HOME;
                    break;
            }
        });
    }
}
