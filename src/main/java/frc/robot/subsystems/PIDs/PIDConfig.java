package frc.robot.subsystems.PIDs;

public class PIDConfig {
    
    private double Kp;
    private double Ki;
    private double Kd;
    private double iZone;
    private double Kf;

    private PIDConfig(double Kp, double Ki, double Kd, double Kf, double iZone){
        this.Kd = Kd;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        this.iZone = iZone;
    }

    public PIDConfig(double Kp, double Ki, double Kd, double Kf){
        this(Kp, Ki, Kd, Kf, 0);
    }

    public PIDConfig(double Kp, double Ki, double Kd){
        this(Kp, Ki, Kd, 0, 0);
    }
}
