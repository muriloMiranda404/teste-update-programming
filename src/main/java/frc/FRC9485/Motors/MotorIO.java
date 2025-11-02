package frc.FRC9485.Motors;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public interface MotorIO {

    @AutoLog
    public static class MotorIOInputs{
        public double position = 0;
        public double speed = 0;
        public double temperature = 0;
        public boolean isInverted = false;
        public double voltage = 0;
    }
    
    double getVoltage();

    double getPosition();

    double getSpeed();

    boolean atSetpoint(double setpoint);

    RelativeEncoder getAlternativeEncoder(boolean usingRelativeEncoder);

    AbsoluteEncoder getAbsoluteEncoder(boolean usingAbsoluteEncoder);

    void setSpeed(double speed);

    void setVoltage(double voltage);

    void FollowMotor(int id, boolean inverted);

    double getMotorTemperature();

    double getMotorOutput();

    SparkMax getSpark();

    int getMotorId();
    
    void setPosition(double position);

    String getMotorName();

    boolean usingInternalMotor();

    void setPID(double Kp, double Ki, double Kd);

    void clearStickyFaults();

    void setReferencePosition(double position);

    void setRampRate(double ramp);

    void updateConfig(SparkMaxConfig config);

    void setMotorInvert(boolean invert);

    boolean motorIsInverted();

    void updateInputs(MotorIOInputs input);
}
