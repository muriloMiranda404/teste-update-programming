package frc.robot.commands.swerveUtils;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightConfig;

public class AlingToTarget extends Command {

    private final LimelightConfig limelight;
    private final SwerveSubsystem swerve;
    private final HolonomicDriveController controller;
    private final Timer timer;

    private static final double FILTRO_ALPHA = 0.2;
    private static final double MAX_SPEED = 0.7;
    private static final double TIMEOUT = 7.0;

    private double txFiltrado = 0.0;
    private double tyFiltrado = 0.0;

    public AlingToTarget() {
        this.limelight = LimelightConfig.getInstance();
        this.swerve = SwerveSubsystem.getInstance();

        PIDController xPID = new PIDController(0.35, 0.0, 0.002);
        PIDController yPID = new PIDController(0.35, 0.0, 0.002);
        ProfiledPIDController rotPID = new ProfiledPIDController(
            0.1, 0.0, 0.002, 
            new Constraints(Math.PI, Math.PI)
        );

        xPID.setTolerance(0.02);
        yPID.setTolerance(0.02);
        rotPID.setTolerance(Math.toRadians(2));

        this.controller = new HolonomicDriveController(xPID, yPID, rotPID);
        this.timer = new Timer();

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        txFiltrado = 0.0;
        tyFiltrado = 0.0;
    }

    @Override
    public void execute() {
        if (!limelight.getHasTarget()) {
            swerve.drive(new Translation2d(), 0, true);
            return;
        }

        double tx = limelight.getTx();
        double ty = limelight.getTy();

        if (Double.isNaN(tx) || Double.isNaN(ty)) {
            System.out.println("Valores inválidos da Limelight!");
            return;
        }

        txFiltrado = FILTRO_ALPHA * tx + (1 - FILTRO_ALPHA) * txFiltrado;
        tyFiltrado = FILTRO_ALPHA * ty + (1 - FILTRO_ALPHA) * tyFiltrado;

        Pose2d currentPose = swerve.getPose();
        Pose2d targetPose = new Pose2d(
            currentPose.getX() + tyFiltrado * 0.05, 
            currentPose.getY() + txFiltrado * 0.05, 
            Rotation2d.fromDegrees(0)
        );

        // Calcula velocidades desejadas
        Trajectory.State targetState = new Trajectory.State();
        targetState.poseMeters = targetPose;
        targetState.velocityMetersPerSecond = 0.0;
        targetState.accelerationMetersPerSecondSq = 0.0;
        
        ChassisSpeeds speeds = controller.calculate(currentPose, targetState, Rotation2d.fromDegrees(0));
        
        double vx = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, speeds.vxMetersPerSecond));
        double vy = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, speeds.vyMetersPerSecond));
        double omega = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, speeds.omegaRadiansPerSecond));

        swerve.drive(new Translation2d(vx, vy), omega, true);
    }

    @Override
    public boolean isFinished() {
        return controller.atReference() || timer.hasElapsed(TIMEOUT);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0, true);
        timer.stop();
        if (interrupted) System.out.println("Alinhamento interrompido!");
        else System.out.println("Alinhamento concluído!");
    }
}
