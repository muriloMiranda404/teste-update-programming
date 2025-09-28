package frc.robot.commands.swerveUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightConfig;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AlingToTarget extends Command {

    private final LimelightConfig limelight;
    private final SwerveSubsystem subsystem;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    private double setpointX, setpointY;

    private final Timer timer = new Timer();
    private boolean automaticSetpoint;

    // PID Constants - aumentadas para resposta mais rápida
    private static final double kP_ROTATION = 0.2;
    private static final double kI_ROTATION = 0.01;
    private static final double kD_ROTATION = 0.005;
    private static final double kP_X = 0.25;
    private static final double kI_X = 0.01;
    private static final double kD_X = 0.005;
    private static final double kP_Y = 0.25;
    private static final double kI_Y = 0.01;
    private static final double kD_Y = 0.005;

    private static final double TOLERANCIA_ROTATION = 0.2;
    private static final double TOLERANCIA_X = 0.05;
    private static final double TOLERANCIA_Y = 0.05;

    private static final double MAX_CORRECAO = 12.0;
    private static final double ZONA_MORTA = 0.02;
    private static final double TIMEOUT_SECONDS = 5.0;
    private static final double TEMPO_MINIMO_ESTAVEL = 0.2;

    private double ultimaTx = 0.0;
    private double ultimoTempoMudanca = 0.0;

    public AlingToTarget(double setpointX, double setpointY) {
        this(setpointX, setpointY, false);
    }

    public AlingToTarget(boolean automaticSetpoint) {
        this(0, 0, true);
    }

    private AlingToTarget(double setpointX, double setpointY, boolean automaticSetpoint) {
        this.limelight = LimelightConfig.getInstance();
        this.subsystem = SwerveSubsystem.getInstance();
        this.setpointX = setpointX;
        this.setpointY = setpointY;
        this.automaticSetpoint = automaticSetpoint;

        this.xController = new PIDController(kP_X, kI_X, kD_X);
        this.yController = new PIDController(kP_Y, kI_Y, kD_Y);
        this.rotationController = new PIDController(kP_ROTATION, kI_ROTATION, kD_ROTATION);

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        ultimaTx = 0.0;
        ultimoTempoMudanca = 0.0;

        if (automaticSetpoint) {
            double[] coords = limelight.getAprilTagCordenates();
            double tx = coords[0];
            double ty = coords[1];

            if (!Double.isNaN(tx) && !Double.isNaN(ty)) {
                setpointX = tx - 0.6;
                setpointY = ty;
            } else {
                System.out.println("Valores inválidos da Limelight, mantendo setpoints existentes.");
            }
        }

        rotationController.reset();
        xController.reset();
        yController.reset();

        rotationController.setSetpoint(0);
        xController.setSetpoint(0);
        yController.setSetpoint(0);

        rotationController.setTolerance(TOLERANCIA_ROTATION);
        xController.setTolerance(TOLERANCIA_X);
        yController.setTolerance(TOLERANCIA_Y);
    }

    @Override
    public void execute() {
        boolean hasTarget = limelight.getHasTarget();

        if (!hasTarget) {
            subsystem.drive(new Translation2d(), 0, true);
            return;
        }

        double tx = limelight.getTx();
        double ty = limelight.getTy();
        double ta = limelight.getTa();

        if (Double.isNaN(tx) || Double.isNaN(ty) || Double.isNaN(ta)) return;

        if (Math.abs(tx - ultimaTx) > 10.0) {
            ultimoTempoMudanca = timer.get();
        }
        ultimaTx = tx;

        if (timer.get() - ultimoTempoMudanca > TEMPO_MINIMO_ESTAVEL) {
            double correcaoRot = rotationController.calculate(tx, setpointX);
            double correcaoX = xController.calculate(ty, setpointY);
            double correcaoY = yController.calculate(ty, setpointY);

            // Zona morta
            if (Math.abs(correcaoRot) < ZONA_MORTA) correcaoRot = 0;
            if (Math.abs(correcaoX) < ZONA_MORTA) correcaoX = 0;
            if (Math.abs(correcaoY) < ZONA_MORTA) correcaoY = 0;

            // Limite máximo de velocidade
            correcaoRot = Math.min(Math.max(correcaoRot, -MAX_CORRECAO), MAX_CORRECAO);
            correcaoX = Math.min(Math.max(correcaoX, -MAX_CORRECAO), MAX_CORRECAO);
            correcaoY = Math.min(Math.max(correcaoY, -MAX_CORRECAO), MAX_CORRECAO);

            // Normaliza ta
            double factor = Math.max(ta, 0.5);

            Translation2d translation = new Translation2d(correcaoX * factor, correcaoY * factor);
            subsystem.drive(translation, -correcaoRot, true);
        }
    }

    @Override
    public boolean isFinished() {
        return (rotationController.atSetpoint() && xController.atSetpoint() && yController.atSetpoint()) 
               || timer.hasElapsed(TIMEOUT_SECONDS);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.drive(new Translation2d(), 0, true);
        timer.stop();
        if (interrupted) {
            System.out.println("Alinhamento interrompido!");
        } else {
            System.out.println("Alinhamento concluído!");
        }
    }
}
