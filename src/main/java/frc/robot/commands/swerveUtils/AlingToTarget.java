package frc.robot.commands.swerveUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FRC9485.vision.LimelightHelpers;
import frc.robot.GeralConstants.vision;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AlingToTarget extends Command{

    private SwerveSubsystem swerveSubsystem;
    
    private Timer dontSeeTag, maxAling;

    private double tagid;

    PIDController xController, yController, rotationController;

    public AlingToTarget(){
        this.swerveSubsystem = SwerveSubsystem.getInstance();
        this.xController = new PIDController(0.01, 0, 0);
        this.yController = new PIDController(0.01, 0, 0);
        this.rotationController = new PIDController(0.01, 0, 0);
    }

    @Override
    public void initialize() {
        this.dontSeeTag = new Timer();
        dontSeeTag.start();

        this.maxAling = new Timer();
        maxAling.start();

        xController.setSetpoint(0);
        yController.setSetpoint(0);
        rotationController.setSetpoint(0);

        xController.setTolerance(vision.X_TOLERANCE);
        yController.setTolerance(vision.Y_TOLERANCE);
        rotationController.setTolerance(vision.ROTATION_TOLERANCE);

        if(LimelightHelpers.getTV("")){
            this.tagid = LimelightHelpers.getFiducialID("");
        }
    }

    @Override
    public void execute() {
        if(LimelightHelpers.getTV("") && tagid == LimelightHelpers.getFiducialID("")){
            dontSeeTag.reset();

            double[] position = LimelightHelpers.getBotPose_TargetSpace("");

            double pidx = xController.calculate(position[2]);
            double pidy = yController.calculate(position[0]);
            double rotation = rotationController.calculate(position[4]); 

            swerveSubsystem.drive(new Translation2d(pidx, pidy), rotation, true);
        } else{
            swerveSubsystem.drive(new Translation2d(0, 0), 0, true);
        }
    }

    @Override
    public boolean isFinished() {
    return !LimelightHelpers.getTV("") || dontSeeTag.hasElapsed(vision.DONT_SEE_TAG) ||
            maxAling.hasElapsed(vision.MAX_ALINGMENT_TIME) || xController.atSetpoint() && yController.atSetpoint() 
            && rotationController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        if(!LimelightHelpers.getTV("")){
            System.out.println("Tag não foi identificada");
            swerveSubsystem.drive(new Translation2d(0, 0), 0, true);
        } else if(dontSeeTag.hasElapsed(vision.DONT_SEE_TAG)){
            System.out.println("Tempo maximo sem ver a tag foi atingido");
            swerveSubsystem.drive(new Translation2d(0, 0), 0, true);
        } else if(maxAling.hasElapsed(vision.MAX_ALINGMENT_TIME)){
            swerveSubsystem.drive(new Translation2d(0, 0), 0, true);
            System.out.println("tempo maximo ultrapassado");
        } else {
            System.out.println("comando bem executado");
        }
    }
}