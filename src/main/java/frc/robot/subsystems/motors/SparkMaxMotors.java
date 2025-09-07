package frc.robot.subsystems.motors;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;

public class SparkMaxMotors implements SparkMaxMotorsIO{
    
    private int id;
    private boolean usingInternalEncoder;
    private String name;

    private SparkMaxConfig config;

    private SparkMax motor;

    private PIDController controller;
    private PIDConstants constants;

    public SparkMaxMotors(int id, boolean usingInternalEncoder, String name){
        this.name = name;
        this.id = id;
        this.usingInternalEncoder = usingInternalEncoder;
        this.config = new SparkMaxConfig();
        this.constants = new PIDConstants(0, 0, 0);
        this.controller = new PIDController(constants.kP, constants.kI, constants.kD);
        this.motor = new SparkMax(id, SparkMax.MotorType.kBrushless);
    }

    @Override
    public double getVoltage() {
       return motor.getAppliedOutput() * motor.getBusVoltage();
    }

    @Override
    public double getSpeed() {
        return motor.get();
    }

    @Override
    public RelativeEncoder getAlternativeEncoder(boolean usingRelativeEncoder) {
        if(usingRelativeEncoder){
            return motor.getAlternateEncoder();
        } else{
        return null;
    }
    }

    @Override
    public boolean usingInternalMotor() {
        return usingInternalEncoder;
    }
    
    @Override
    public AbsoluteEncoder getAbsoluteEncoder(boolean usingAbsoluteEncoder) {
       if(usingAbsoluteEncoder){
        return motor.getAbsoluteEncoder();
       } else{
       return null;
    }
    }

    @Override
    public String getMotorName() {
        return name;
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public int getMotorId() {
        return id;
    }

    @Override
    public void setVoltage(double voltage) {
       motor.setVoltage(voltage);
    }

    @Override
    public void FollowMotor(int id, boolean inverted) {
    config.follow(id);
    config.inverted(inverted);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public double getMotorTemperature() {
       return motor.getMotorTemperature();
    }

    @Override
    public double getMotorOutput(){
        return motor.getAppliedOutput();
    }

    @Override
    public SparkMax getSpark(){
        return motor;
    }

    @Override
    public void setPID(double Kp, double Ki, double Kd){
        controller.setP(Kp);
        controller.setI(Ki);
        controller.setD(Kd);
    }
}
