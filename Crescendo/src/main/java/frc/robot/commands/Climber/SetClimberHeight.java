package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class SetClimberHeight extends Command {

    private Climber climber;

    public SetClimberHeight(Climber climber) {
        this.climber = climber;
    }

}