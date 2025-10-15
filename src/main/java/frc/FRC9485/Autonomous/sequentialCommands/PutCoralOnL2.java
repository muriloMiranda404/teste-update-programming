package frc.FRC9485.Autonomous.sequentialCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.level.intake.SetIntakeSpeed;
import frc.robot.subsystems.Mechanism.SuperStructure;
import frc.robot.subsystems.Mechanism.SuperStructure.StatesToScore;
import frc.robot.subsystems.Mechanism.intake.IntakeSubsystem;

public class PutCoralOnL2 extends SequentialCommandGroup{

    IntakeSubsystem intakeSubsystem;
    SuperStructure superStructure;

    public PutCoralOnL2(IntakeSubsystem intakeSubsystem, SuperStructure superStructure){
        this.intakeSubsystem = intakeSubsystem;
        this.superStructure = superStructure;

        addCommands(
            superStructure.scorePieceOnLevel(StatesToScore.L2).onlyIf(() -> !superStructure.scoreIsFinised()),
            new ThrowCoralAndGet(intakeSubsystem, superStructure)
        );
    }
}