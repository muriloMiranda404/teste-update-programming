package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;

public class IntakeSubsystem extends SubsystemBase{
    
    SparkMax turnIntake;
    SparkMax getCoral;

    DutyCycleEncoder encoder;

    PIDController controller;

    DigitalInput coralswitch;

    public static IntakeSubsystem mInstance = new IntakeSubsystem();

    private IntakeSubsystem(){

        this.turnIntake = new SparkMax(Intake.INTAKE_MOTOR, SparkMax.MotorType.kBrushless);
        this.getCoral = new SparkMax(Intake.CORAL_MOTOR, SparkMax.MotorType.kBrushless);

        this.encoder = new DutyCycleEncoder(Intake.INTAKE_ENCODER);
        this.encoder.setDutyCycleRange(0, 360);

        this.controller = Intake.INTAKE_PID;
        this.controller.setTolerance(Intake.INTAKE_TOLERANCE);

        this.coralswitch = new DigitalInput(Intake.CORAL_SWITCH);

    }

    public static IntakeSubsystem getInstance(){
        if(mInstance == null){
            return new IntakeSubsystem();
        }
        return mInstance;
    }

    public void setSpeed(double speed){
        getCoral.set(speed);
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
    
    public void setPosition(double setpoint){
        double ang = getDistance();

        if(setpoint < 55.0){

            setpoint = 55.0;
        
        } else if(setpoint > 230.0){
         
            setpoint = 230.0;
        
        }

        double output = controller.calculate(ang, setpoint);
        
        turnIntake.set(output);
        System.out.println("setpoint: " + setpoint);
    }

    public boolean atSetpoint(){
        return controller.atSetpoint();
    }

    public double getCoralMotorVoltage(){
        return getCoral.getBusVoltage();
    }

    public double getCoralMotorTemperature(){
        return getCoral.getMotorTemperature();
    }

    public double getOutputInCoralMotor(){
        return getCoral.getAppliedOutput();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("angulo", getDistance());
        SmartDashboard.putBoolean("fim de curos do coral", IsTouched());
    }
}
