package frc.robot.subsystems.Mechanism.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Intake.IntakePositions;
import frc.robot.subsystems.Mechanism.MechanismIO;
import frc.robot.subsystems.Motors.MotorIO;
import frc.robot.subsystems.Motors.SparkMaxMotors;

public class IntakeSubsystem extends SubsystemBase implements MechanismIO{
    
    public MotorIO turnIntake;
    public MotorIO getCoral;
 
    public DutyCycleEncoder encoder;
 
    public PIDController controller;
 
    public DigitalInput coralswitch;

    public double setpoint;
    public double speed;
    
    public static IntakeSubsystem mInstance = null;

    public boolean hasCoral;

    private IntakeSubsystem(){

        this.turnIntake = new SparkMaxMotors(Intake.INTAKE_MOTOR, false, "Turn intake motor");
        this.getCoral = new SparkMaxMotors(Intake.CORAL_MOTOR, true, "get game piece motor");

        this.encoder = new DutyCycleEncoder(Intake.INTAKE_ENCODER);

        this.controller = Intake.INTAKE_PID;

        this.coralswitch = new DigitalInput(Intake.CORAL_SWITCH);

        this.setpoint = 0;
        this.speed = 0;

        this.hasCoral = false;

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

        this.controller.setTolerance(Intake.INTAKE_TOLERANCE);
    }

    public boolean IsTouched(){
        hasCoral = true;
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
        return encoder.get() * 360.0;
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
        return controller.atSetpoint();
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
            getCoral.setSpeed(Intake.GET_CORAL_SPEED);
        }
        getCoral.setSpeed(0);
    }

    public boolean hasCoralOnIntake(){
        return hasCoral;
    }

    public double getSpeedOnIntakeMotor(){
        return speed;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("angulo", getDistance());
        SmartDashboard.putBoolean("fim de curos do coral", IsTouched());
    }
}