package frc.robot.commands.belt;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Belt;

public class ManualSetBelt extends Command {
    private double speed;
    private final Belt belt;

    public ManualSetBelt(Belt belt, double speed) {
        this.belt = belt;
        this.speed = speed;
        addRequirements(belt);
    }

    @Override
    public void execute() {
        belt.moveDoubleBelt(speed);
    }
}
