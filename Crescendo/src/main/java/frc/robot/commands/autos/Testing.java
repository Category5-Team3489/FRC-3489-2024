package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutonomousDrive;
import frc.robot.commands.autoShooting.AutonomousShoot;

public class Testing {
    AutonomousDrive autoDrive1 = new AutonomousDrive(1, 2, 1);
    AutonomousDrive autoDrive2 = new AutonomousDrive(1, 2, 1);

    AutonomousShoot autoShoot1 = new AutonomousShoot();

    public Command testing() {
        // return autoDrive1.withTimeout(2)
        // .andThen(() -> autoDrive1.cancel())

        // .andThen(autoDrive2.withTimeout(2))
        // .andThen(() -> autoDrive2.cancel())
        // .andThen(Commands.print("END OF DRIVE"))
        // .andThen(autoShoot1)
        // .andThen(Commands.print("AUTO SHOOT DONE"))
        // .withName("TESTING");

        return autoDrive1.withTimeout(2)
                .andThen(autoShoot1)
                .andThen(Commands.print("AUTO SHOOT DONE"))
                .withName("TESTING");

    }

}
