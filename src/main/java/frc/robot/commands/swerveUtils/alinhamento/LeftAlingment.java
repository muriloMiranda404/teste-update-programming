package frc.robot.commands.swerveUtils.alinhamento;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FRC9485.constants.AlingConstants;
import frc.FRC9485.vision.LimelightHelpers;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class LeftAlingment extends Command{
    
    private SwerveSubsystem swerveSubsystem;
    private PIDController X, Y, rotation;
    private Timer dontSeeTag, limit;

    private double[] position;

    public LeftAlingment(){
        this.swerveSubsystem = SwerveSubsystem.getInstance();
        this.X = new PIDController(0.01, 0, 0);
        this.Y = new PIDController(0.01, 0, 0);
        this.rotation = new PIDController(0.01, 0, 0);

        this.limit = new Timer();
        this.dontSeeTag = new Timer();

        addRequirements(swerveSubsystem);

        // position[0] = X do robô em relação ao alvo (metros)
        // position[1] = Y do robô em relação ao alvo (metros)
        // position[2] = Z (altura)
        // position[3] = rotação roll
        // position[4] = pitch
        // position[5] = yaw (ângulo para o alvo) -> isso é target_space
    }

    @Override
    public void initialize() {
        //Todo que estão comentados devem ser mudados para os valores respectivos
        
        this.rotation.reset();
        this.rotation.setTolerance(AlingConstants.ROTATION_TOLERANCE);
        this.rotation.enableContinuousInput(-180, 180);
        this.rotation.setSetpoint(0); // alinhar reto com a tag
        
        this.X.reset();
        this.X.setTolerance(AlingConstants.X_TOLERANCE);
        this.X.setSetpoint(0); //alinhar no centro da tag

        this.Y.reset();        
        this.Y.setTolerance(AlingConstants.Y_TOLERANCE);
        this.Y.setSetpoint(AlingConstants.LEFT_OFFSET); // alinhar distancia de lado -> direita significa que ele deve ir um pouco mais ao lado

        this.limit.reset();
        this.limit.start();
    }
    
    @Override
    public void execute() {
        if(LimelightHelpers.getTV("")){

            this.position = LimelightHelpers.getBotPose_TargetSpace(""); // pose do robo em metros
            this.dontSeeTag.reset();

            double normalize = Math.toDegrees(position[5]);

            double Xcontroller = X.calculate(position[0]);
            double Ycontroller = Y.calculate(position[1]);
            double omega = rotation.calculate(normalize); // a pose é em metros, já o alinhamento é em centimetros
            
            if(!Double.isNaN(Xcontroller) && !Double.isNaN(Ycontroller) && !Double.isNaN(omega)){
                swerveSubsystem.drive(new Translation2d(Xcontroller, Ycontroller), omega, true);
            }
        } else {
            this.cancel();
            DriverStation.reportError("O alinhamento não pode ser concluido devido a ausencia da tag", false);
        }
    }

    @Override
    public boolean isFinished() {
        return X.atSetpoint() && Y.atSetpoint() && rotation.atSetpoint() ||
        dontSeeTag.hasElapsed(AlingConstants.DONT_SEE_TAG) ||
        limit.hasElapsed(AlingConstants.MAX_ALINGMENT_TIME);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new Translation2d(0, 0), 0, true);
    }
}
