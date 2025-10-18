package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.FRC9485.Motors.SparkMaxMotors;

public class SwerveModule {
    
    private SparkMaxMotors Drive;
    private SparkMaxMotors Angle;
    private CANcoder CANcoder;
    private String module;

    public SwerveModule(int Drive, int Angle, int CANcoder, String module){
        this.Drive = new SparkMaxMotors(Drive, false, null);
        this.Angle = new SparkMaxMotors(Angle, false, null);
        this.CANcoder = new CANcoder(CANcoder);
        this.module = module;
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(Drive.getPosition(), Rotation2d.fromRadians(Angle.getPosition()));
    }

    public void setDesiredState(SwerveModuleState state, boolean isPID){
    
        Rotation2d atual = getPosition().angle;

        state.optimize(atual);
    
        Drive.setSpeed(state.speedMetersPerSecond);

        double anguloOutput = state.angle.getRadians();
        double current = atual.getRadians();
        double erro = anguloOutput - current;

        double entrada = Math.IEEEremainder(erro, 2 * Math.PI);

        Angle.setSpeed(entrada);
    }
}
