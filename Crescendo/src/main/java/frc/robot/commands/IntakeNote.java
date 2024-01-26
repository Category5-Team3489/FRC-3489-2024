package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class IntakeNote extends SequentialCommandGroup {

    public IntakeNote() {
        //addCommands(
                //Commands.runOnce());
    }

}

// start intake/belt
// wait to get sensor value (could have this start shooter motor)
// stop