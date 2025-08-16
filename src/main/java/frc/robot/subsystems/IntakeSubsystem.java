package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
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

    public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    private IntakeSubsystem(){

        turnIntake = new SparkMax(Intake.INTAKE_MOTOR, SparkMax.MotorType.kBrushless);
        getCoral = new SparkMax(Intake.CORAL_MOTOR, SparkMax.MotorType.kBrushless);

        encoder = new DutyCycleEncoder(Intake.INTAKE_ENCODER);
        encoder.setDutyCycleRange(0, 360);

        controller = Intake.INTAKE_PID;
        controller.setTolerance(Intake.INTAKE_TOLERANCE);

        coralswitch = new DigitalInput(Intake.CORAL_SWITCH);

    }

    public static IntakeSubsystem getInstance(){
        if(intakeSubsystem == null){
            return new IntakeSubsystem();
        }
        return intakeSubsystem;
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
        
        // if(ang < 55.0){
        //     if(output < 0.0) output = 0.0;
            
        //     if(setpoint < 55.0) setpoint = 55.0;
        // }

        // if(ang > 230.0){
        //     if(output > 0.0) {
        //         output = 0.0;
        //     }
            
        //     if(setpoint > 230.0) {
        //         setpoint = 230.0;
        //     }
        // }
        
        turnIntake.set(output);
        System.out.println("setpoint: " + setpoint);
    }

    public boolean atSetpoint(){
        return controller.atSetpoint();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("angulo", getDistance());
        SmartDashboard.putBoolean("fim de curos do coral", IsTouched());
    }
}
