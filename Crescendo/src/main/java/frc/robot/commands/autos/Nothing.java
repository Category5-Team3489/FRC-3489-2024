package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;

public class Nothing extends Command {

    public Nothing() {}

    @Override
    public void initialize() {
        System.out.println("Doing nothing... Because this is the nothing auto!");
    }

}
