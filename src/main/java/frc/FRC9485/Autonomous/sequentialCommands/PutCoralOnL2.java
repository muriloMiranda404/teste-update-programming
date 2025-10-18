package frc.FRC9485.Autonomous.sequentialCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
            superStructure.scorePieceOnLevel(StatesToScore.L2).onlyIf(() -> intakeSubsystem.IsTouched()),
            new ThrowCoralAndGet(intakeSubsystem, superStructure)
        );
    }
}