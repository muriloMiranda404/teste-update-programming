package frc.robot.subsystems.Mechanism.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FRC9485.Motors.MotorIO;
import frc.FRC9485.Motors.SparkMaxMotors;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.Mechanism.MechanismIO;

public class ElevatorSubsystem extends SubsystemBase implements MechanismIO{
    
    private MotorIO rightMotor;
    private MotorIO leftMotor;

    private DigitalInput downSwitch;
    private DigitalInput upSwitch;

    private PIDController controller;

    private Encoder encoder;

    public static ElevatorSubsystem mInstance = null;

    public double setpoint;

    
    private ElevatorSubsystem(){
        
        this.rightMotor = new SparkMaxMotors(Elevator.RIGHT_ELEVATOR_MOTOR, true, "right elevator motor");
        this.leftMotor = new SparkMaxMotors(Elevator.LEFT_ELEVATOR_MOTOR, false, "left elevator motor");

        this.downSwitch = new DigitalInput(Elevator.DOWN_SWITCH);
        this.upSwitch = new DigitalInput(Elevator.UP_SWITCH);

        this.encoder = new Encoder(Elevator.ENCODER_ELEV_A, Elevator.ENCODER_ELEV_B);
        
        this.controller = Elevator.ELEVATOR_PID;

        this.configureElevator();
    }
    
    public static ElevatorSubsystem getInstance(){
        if(mInstance == null){
            mInstance = new ElevatorSubsystem();
        }
        return mInstance;
    }
    
    public void configureElevator(){
        this.encoder.setDistancePerPulse(360.0/2048.0);

        this.controller.setTolerance(Elevator.ELEVATOR_TOLERANCE);
    
        this.setpoint = 0;
    }
    
    @Override
    public double getDistance(){
        return encoder.getDistance();
    }

    public double[] getRateOnMotor(){
        double[] speed = {
            rightMotor.getSpeed(),
            leftMotor.getSpeed()
        };

        return speed;
    }

    @Override
    public double getSetpoint(){
        return setpoint;
    }

    @Override
    public void setPosition(double setpoint){
        double ang = getDistance();
        this.setpoint = setpoint;

        if(setpoint < 0){
            setpoint = 0.0;
        } else if(setpoint > 1480.0){
            setpoint = 1480.0;
        }

        double output = controller.calculate(ang, setpoint) * -1.0;
        
        if(upSwitch.get()){
            if(output > 0){
                output = 0.0;
            }

            if(setpoint > 1480.0){
                setpoint = 1480.0;
            }
        }

        if(downSwitch.get() && output > 0.0){

            if(output < 0){
                output = 0.0;
            }

            if(setpoint < 0){
                setpoint = 0.0;
            }

        }

        rightMotor.setSpeed(output);
        leftMotor.setSpeed(-output);
    }

    @Override
    public boolean atSetpoint(){
        return controller.atSetpoint();
    }

    public void stopElevator(){
    rightMotor.setVoltage(0);
    leftMotor.setVoltage(0);
   }

   public boolean upSwitchGet(){
    boolean trava = false;

    if(upSwitch.get() && trava == false){
        trava = true;
    } else{
        trava = false;
    }

    return trava;
   }

    public double[] getOutputInElevatorMotors(){
        double[] output = {
            rightMotor.getMotorOutput(),
            leftMotor.getMotorOutput()
        };

        return output;
    }

    public double getErroOnElevatorOutput(){
        return this.controller.getError();
    }

    public Command resetElevator(){
        return run(() ->{
            encoder.reset();
        });
    }

    public boolean elevatorIsInCorrectRange(double range){
        return encoder.getDistance() >= range - Elevator.ELEVATOR_TOLERANCE &&
        encoder.getDistance() <= range + Elevator.ELEVATOR_TOLERANCE;
    }

    @Override
    public void setSpeed(double speed){
        rightMotor.setSpeed(speed);
        leftMotor.setSpeed(speed);
    }

    @Override
    public void periodic() {}
}
