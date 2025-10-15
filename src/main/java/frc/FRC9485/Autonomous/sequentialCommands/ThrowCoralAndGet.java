package frc.FRC9485.Autonomous.sequentialCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.level.intake.SetIntakeSpeed;
import frc.robot.subsystems.Mechanism.SuperStructure;
import frc.robot.subsystems.Mechanism.SuperStructure.StatesToScore;
import frc.robot.subsystems.Mechanism.intake.IntakeSubsystem;

public class ThrowCoralAndGet extends SequentialCommandGroup{
    
    IntakeSubsystem intakeSubsystem;
    SuperStructure superStructure;

    public ThrowCoralAndGet(IntakeSubsystem intakeSubsystem, SuperStructure superStructure){
        this.intakeSubsystem = intakeSubsystem;
        this.superStructure = superStructure;

        addCommands(
            new SetIntakeSpeed(0.7).onlyWhile(() -> intakeSubsystem.IsTouched()),
            new ParallelCommandGroup(
                superStructure.scorePieceOnLevel(StatesToScore.L1),
                new SetIntakeSpeed().onlyIf(() -> !intakeSubsystem.IsTouched())
            )
        );
    }
}
