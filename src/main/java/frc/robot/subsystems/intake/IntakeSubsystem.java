package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.motors.SparkMaxMotors;

public class IntakeSubsystem extends SubsystemBase{
    
    public SparkMaxMotors turnIntake;
    public SparkMaxMotors getCoral;
 
    public DutyCycleEncoder encoder;
 
    public PIDController controller;
 
    public DigitalInput coralswitch;

    public double setpoint;

    public static IntakeSubsystem mInstance = null;

    private IntakeSubsystem(){

        this.turnIntake = new SparkMaxMotors(Intake.INTAKE_MOTOR, false, "Turn intake motor");
        this.getCoral = new SparkMaxMotors(Intake.CORAL_MOTOR, true, "get game piece motor");

        this.encoder = new DutyCycleEncoder(Intake.INTAKE_ENCODER);
        this.encoder.setDutyCycleRange(0, 360);

        this.controller = Intake.INTAKE_PID;
        this.controller.setTolerance(Intake.INTAKE_TOLERANCE);

        this.coralswitch = new DigitalInput(Intake.CORAL_SWITCH);

        this.setpoint = 0;
    }

    public static IntakeSubsystem getInstance(){
        if(mInstance == null){
            mInstance = new IntakeSubsystem();
        }
        return mInstance;
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

    public double getDistance(){
        return encoder.get() * 360.0;
    }

    public double getSetpoint(){
        return setpoint;
    }
    
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

    public boolean atSetpoint(){
        return controller.atSetpoint();
    }

    public double getCoralMotorVoltage(){
        return getCoral.getVoltage();
    }

    public double getCoralMotorTemperature(){
        return getCoral.getMotorTemperature();
    }

    public void getCoralOnIntake(){
        if(!IsTouched()){
            getCoral.setSpeed(0.2);
        }
        getCoral.setSpeed(0);
    }
    
    public void set(double speed) {
        getCoral.setSpeed(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("angulo", getDistance());
        SmartDashboard.putBoolean("fim de curos do coral", IsTouched());
    }
}