package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.IntakeUntilDetection;
import frc.robot.enums.IntakeState;
import frc.robot.subsystems.Index;

public class IntakeNote extends SequentialCommandGroup {

    public IntakeNote(Index belt, IntakeState intakeState) {
        addCommands(Commands.parallel(

        ));
        // addCommands(
        // Commands.runOnce());
    }

}

// start intake/belt
// wait to get sensor value (could have this start shooter motor)
// stop