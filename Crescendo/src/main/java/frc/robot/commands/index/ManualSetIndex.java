package frc.robot.commands.index;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index;

public class ManualSetIndex extends Command {
    private double speed;
    private final Index belt;

    public ManualSetIndex(Index belt, double speed) {
        this.belt = belt;
        this.speed = speed;
        addRequirements(belt);
    }

    @Override
    public void execute() {
        //belt.moveDoubleIndex(speed);
    }

}
