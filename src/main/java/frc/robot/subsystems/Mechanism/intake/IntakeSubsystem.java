package frc.robot.subsystems.Mechanism.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FRC9485.Motors.MotorIO;
import frc.FRC9485.Motors.SparkMaxMotors;
import frc.FRC9485.Motors.MotorIO.MotorIOInputs;
import frc.FRC9485.constants.IntakeConstants;
import frc.FRC9485.constants.IntakeConstants.IntakePositions;
import frc.FRC9485.controllers.MechanismJoystick;
import frc.robot.subsystems.Mechanism.MechanismIO;

public class IntakeSubsystem extends SubsystemBase implements MechanismIO, LoggableInputs{
    
    private MotorIO turnIntake;
    private MotorIO getCoral;
 
    private DutyCycleEncoder encoder;
 
    private PIDController controller;
 
    private DigitalInput coralswitch;

    private double setpoint;
    private double speed;

    private MotorIOInputs throwAndCatchCoralInput;
    private MotorIOInputs turnIntakeIOInput;
    
    public static IntakeSubsystem mInstance = null;

    private boolean hasCoral;
    private double distance;
    private boolean atSetpoint;

    private final MechanismJoystick mechanismJoystick;

    private IntakeSubsystem(){

        this.turnIntake = new SparkMaxMotors(IntakeConstants.INTAKE_MOTOR, false, "Turn intake motor");
        this.getCoral = new SparkMaxMotors(IntakeConstants.CORAL_MOTOR, true, "get game piece motor");

        this.encoder = new DutyCycleEncoder(IntakeConstants.INTAKE_ENCODER);
        
        this.controller = IntakeConstants.INTAKE_PID;
        
        this.coralswitch = new DigitalInput(IntakeConstants.CORAL_SWITCH);
        
        this.mechanismJoystick = MechanismJoystick.getInstance();
        
        this.throwAndCatchCoralInput = new MotorIOInputs();
        this.turnIntakeIOInput = new MotorIOInputs();

        configureIntake();
    }

    public static IntakeSubsystem getInstance(){
        if(mInstance == null){
            mInstance = new IntakeSubsystem();
        }
        return mInstance;
    }
    
    private void configureIntake(){
        this.encoder.setDutyCycleRange(0, 360);
        
        this.controller.setTolerance(IntakeConstants.INTAKE_TOLERANCE);
        
        this.setpoint = 0;
        this.speed = 0;
        
        this.hasCoral = !coralswitch.get();
        this.distance = 0;
        this.atSetpoint = false;
    }

    public boolean IsTouched(){
        return !coralswitch.get();
    }

    public void stopCoralMotor(){
        getCoral.setVoltage(0);
    }
    public void stopIntakeMotor(){
        turnIntake.setVoltage(0);
    }

    @Override
    public double getDistance(){
        this.distance = encoder.get() * 360;
        
        return distance;
    }

    @Override
    public double getSetpoint(){
        return setpoint;
    }
    
    @Override
    public void setPosition(double setpoint){
        double ang = getDistance();
        this.setpoint = setpoint;

        if(setpoint < 55.0){

            setpoint = 55.0;
        
        } else if(setpoint > 230.0){
         
            setpoint = 230.0;
        
        }

        double output = controller.calculate(ang, setpoint);
        
        turnIntake.setSpeed(output);
        System.out.println("setpoint: " + setpoint);
    }

    @Override
    public boolean atSetpoint(){
        this.atSetpoint = controller.atSetpoint();

        return atSetpoint;
    }

    public double getCoralMotorVoltage(){
        return getCoral.getVoltage();
    }

    public double getCoralMotorTemperature(){
        return getCoral.getMotorTemperature();
    }
    
    @Override
    public void setSpeed(double speed) {
        this.speed = speed;
        getCoral.setSpeed(speed);
    }

    public void goToGetCoralPosition(){
        if(!hasCoral){
            turnIntake.setReferencePosition(IntakePositions.DEFAULT_POSITION);
        }
    }

    public void getCoral(){
        if(!hasCoral){
            getCoral.setSpeed(IntakeConstants.GET_CORAL_SPEED);
        }
        getCoral.setSpeed(0);
    }

    public boolean hasCoralOnIntake(){
        return hasCoral;
    }

    public double getSpeedOnIntakeMotor(){
        return speed;
    }

    public Command throwCoral(){
        return run(() ->{
           setSpeed( mechanismJoystick.getRightTrigger() - mechanismJoystick.getLeftTrigger());
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("distance", getDistance());
        getCoral.updateInputs(throwAndCatchCoralInput);
        turnIntake.updateInputs(turnIntakeIOInput);
    }

    @Override
    public void toLog(LogTable table) {
        table.put("speed", speed);
        table.put("hasCoral", hasCoral);
        table.put("AtSetpoint", atSetpoint);
        table.put("setpoint", setpoint);
        table.put("Distance", distance);
        table.put("isTouched", hasCoral);
    }

    @Override
    public void fromLog(LogTable table) {
        speed = table.get("speed", speed);
        hasCoral = table.get("hasCoral", hasCoral);
        atSetpoint = table.get("atSepoint", atSetpoint);
        setpoint = table.get("setpoint", setpoint);
        distance = table.get("distance", distance);
        hasCoral = table.get("isTouched", hasCoral);
    }
}