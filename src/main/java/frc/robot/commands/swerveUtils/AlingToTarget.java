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

    private static final double kP_ROTATION = 0.1;
    private static final double kI_ROTATION = 0.0;
    private static final double kD_ROTATION = 0.0;

    private static final double kP_X = 0.35;
    private static final double kI_X = 0.0;
    private static final double kD_X = 0.002;

    private static final double kP_Y = 0.35;
    private static final double kI_Y = 0.0;
    private static final double kD_Y = 0.002;

    private static final double TOLERANCIA_ROTATION = 0.05;
    private static final double TOLERANCIA_X = 0.05;
    private static final double TOLERANCIA_Y = 0.05;

    private static final double MAX_CORRECAO = 0.7;
    private static final double ZONA_MORTA = 0.02;
    private static final double TIMEOUT_SECONDS = 7.0;
    private static final double TEMPO_MINIMO_ESTAVEL = 0.2;

    private double ultimaTx = 0.0;
    private double ultimoTempoMudanca = 0.0;

    private double txFiltrado = 0.0;
    private double tyFiltrado = 0.0;
    private static final double FILTRO_ALPHA = 0.2; 

    public AlingToTarget(double setpointX, double setpointY) {
        this(setpointX, setpointY, false);
    }

    public AlingToTarget() {
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
            if (limelight.getHasTarget()) {
                double[] coords = limelight.getAprilTagCordenates();
                double tx = coords[0];
                double ty = coords[1];
                if (!Double.isNaN(tx) && !Double.isNaN(ty)) {
                    this.setpointX = ty - 0.4; 
                    this.setpointY = tx;      
                } else {
                    System.out.println("Valores inválidos da Limelight, mantendo setpoints existentes.");
                }
            } else {
                System.out.println("Sem alvo detectado na inicialização, aguardando...");
            }
        }

        rotationController.reset();
        xController.reset();
        yController.reset();

        xController.setSetpoint(setpointX);
        yController.setSetpoint(setpointY);
        rotationController.setSetpoint(0);

        rotationController.setTolerance(TOLERANCIA_ROTATION);
        xController.setTolerance(TOLERANCIA_X);
        yController.setTolerance(TOLERANCIA_Y);
    }

    @Override
    public void execute() {

        if (!limelight.getHasTarget()) {
            subsystem.drive(new Translation2d(), 0, true);
            System.out.println("limelight não esta detectando april tag");
            return;
        }

        double tx = limelight.getTx();
        double ty = limelight.getTy();

        if (Double.isNaN(tx) || Double.isNaN(ty)) return;

        txFiltrado = FILTRO_ALPHA * tx + (1 - FILTRO_ALPHA) * txFiltrado;
        tyFiltrado = FILTRO_ALPHA * ty + (1 - FILTRO_ALPHA) * tyFiltrado;

        if (Math.abs(txFiltrado - ultimaTx) > 10.0) {
            ultimoTempoMudanca = timer.get();
        }
        ultimaTx = txFiltrado;

        if (timer.get() - ultimoTempoMudanca > TEMPO_MINIMO_ESTAVEL) {
            double correcaoRot = rotationController.calculate(txFiltrado, 0);
            double correcaoX = xController.calculate(tyFiltrado, setpointX);
            double correcaoY = yController.calculate(0, setpointY);

            if (Math.abs(correcaoRot) < ZONA_MORTA) correcaoRot = 0;
            if (Math.abs(correcaoX) < ZONA_MORTA) correcaoX = 0;
            if (Math.abs(correcaoY) < ZONA_MORTA) correcaoY = 0;

            correcaoRot = Math.min(Math.max(correcaoRot, -MAX_CORRECAO), MAX_CORRECAO);
            correcaoX = Math.min(Math.max(correcaoX, -MAX_CORRECAO), MAX_CORRECAO);
            correcaoY = Math.min(Math.max(correcaoY, -MAX_CORRECAO), MAX_CORRECAO);

            Translation2d translation2d = new Translation2d(correcaoY, -correcaoX);
            subsystem.drive(translation2d, -correcaoRot, true);
        }
    }

    @Override
    public boolean isFinished() {
        return (rotationController.atSetpoint() &&
                xController.atSetpoint() &&
                yController.atSetpoint())
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
