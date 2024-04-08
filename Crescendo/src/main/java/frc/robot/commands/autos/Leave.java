package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveSeconds;
import frc.robot.subsystems.Drivetrain;

public class Leave extends Command {
    private final Drivetrain drivetrain;



    public Leave(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        final DriveSeconds driveSeconds = new DriveSeconds(drivetrain, 3, 0.15);
        driveSeconds.schedule();
        System.out.println("AUTO INIT!");
    }

    
}
