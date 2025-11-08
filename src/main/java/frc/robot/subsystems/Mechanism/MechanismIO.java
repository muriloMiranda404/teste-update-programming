package frc.robot.subsystems.Mechanism;

import edu.wpi.first.wpilibj.util.Color;

public interface MechanismIO {

    public class SuperStructureInput{
        public double elevatorInput = 0;
        public double intakeInput = 0;
        public Color color = null;
        public String state = "";
    }
    
    double getDistance();

    double getSetpoint();

    boolean atSetpoint();

    void setPosition(double setpoint);

    void setSpeed(double speed);
}
