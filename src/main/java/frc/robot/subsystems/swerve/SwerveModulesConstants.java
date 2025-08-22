package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;

public class SwerveModulesConstants {

    public enum ModuleId{
        module1(0),
        module2(1),
        module3(2),
        module4(3);

        Integer id;
        ModuleId(Integer id){
            this.id = id;
        }
    }
    
    public final int DriveId;
    public final int AngleId;
    public final int EncoderId;
    public final Translation2d translation;
    public final ModuleId module;


    public SwerveModulesConstants(ModuleId module, int DriveId, int AngleId, int EncoderId,
    Translation2d translation){
        this.module = module;
        this.DriveId = DriveId;
        this.AngleId = AngleId;
        this.EncoderId = EncoderId;
        this.translation = translation;
    }
}
