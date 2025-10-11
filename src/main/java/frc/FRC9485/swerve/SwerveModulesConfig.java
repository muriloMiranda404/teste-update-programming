package frc.FRC9485.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.FRC9485.Motors.SparkMaxMotors;
import frc.robot.Constants.swerve;

public class SwerveModulesConfig {
    
    SparkMaxMotors driveMotor;
    SparkMaxSim driveSimulation;
    double driveSpeed;
    double drivePosition;

    SparkMaxMotors AngleMotor;
    SparkMaxSim angleSimulation;
    double angleSpeed;
    double anglePosition;

    CANcoder encoder;
    CANcoderSimState encoderState;
    StatusSignal<AngularVelocity> encoderSpeed;
    StatusSignal<Angle> encoderAbsolutePosition;

    Translation2d translation;
    
    SwerveModulesConstants constants;

    int moduleNumber;

    public SwerveModulesConfig(SwerveModulesConstants modules){

        this.moduleNumber = constants.module.id;

        this.driveMotor = new SparkMaxMotors(constants.DriveId, true, "drive motor");
        this.driveSimulation = new SparkMaxSim(driveMotor.getSpark(), DCMotor.getNEO(1));
        this.driveSpeed = driveMotor.getSpark().getEncoder().getVelocity();
        this.drivePosition = driveMotor.getSpark().getEncoder().getPosition();

        this.AngleMotor = new SparkMaxMotors(constants.AngleId, true, "angle motor");
        this.angleSimulation = new SparkMaxSim(AngleMotor.getSpark(), DCMotor.getNEO(1));
        this.angleSpeed = AngleMotor.getSpark().getEncoder().getVelocity();
        this.anglePosition = AngleMotor.getSpark().getEncoder().getPosition();

        this.encoder = new CANcoder(constants.EncoderId);
        this.encoderState = encoder.getSimState();
        this.encoderSpeed = encoder.getVelocity();
        this.encoderAbsolutePosition = encoder.getAbsolutePosition();

        configureDriveMotor();
        configureAngleMotor();
    }

    private void configureDriveMotor(){
        var config = new SparkMaxConfig();
        config.closedLoop.p(swerve.KP_DRIVE);
        config.closedLoop.i(swerve.KI_DRIVE);
        config.closedLoop.d(swerve.KD_DRIVE);
        config.inverted(swerve.IS_INVERTED_DRIVE);

        driveMotor.getSpark().configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private void configureAngleMotor(){
        var configAng = new SparkMaxConfig();
        configAng.closedLoop.p(swerve.KP_ANGLE);
        configAng.closedLoop.i(swerve.KI_ANGLE);
        configAng.closedLoop.d(swerve.KD_ANGLE);
        configAng.inverted(swerve.IS_INVERTED_ANGLE);

        AngleMotor.getSpark().configure(configAng, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            rotationsPerMeter(drivePosition),
            getAngle()
        );
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(anglePosition);
    }

    private double rotationsPerMeter(double rotation){
        return rotation * swerve.ROTATION_PER_METER;
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            rotationsPerMeter(anglePosition), getAngle());
    }
}
