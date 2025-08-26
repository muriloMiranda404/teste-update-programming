package frc.robot.subsystems.motors;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

public interface SparkMaxMotorsIO {
    
    double getVoltage();

    double getSpeed();

    RelativeEncoder getAlternativeEncoder(boolean usingRelativeEncoder);

    AbsoluteEncoder getAbsoluteEncoder(boolean usingAbsoluteEncoder);

    void setSpeed(double speed);

    void setVoltage(double voltage);

    void FollowMotor(int id, boolean inverted);

    double getMotorTemperature();

    double getMotorOutput();

    SparkMax getSpark();

}
