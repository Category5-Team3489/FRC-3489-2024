package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.commands.AutonomousDrive;
import frc.robot.commands.Intake.AutonomousCoralIntake;
import frc.robot.commands.autoShooting.AutonomousShoot;
import frc.robot.subsystems.Drivetrain;

public class Testing extends SequentialCommandGroup {

    AutonomousDrive autoDrive1 = new AutonomousDrive(1, 2, 1);
    AutonomousDrive autoDrive2 = new AutonomousDrive(1, 2, 1);

    AutonomousShoot autoShoot1 = new AutonomousShoot();

    AutonomousCoralIntake autoIntake = new AutonomousCoralIntake();

    public Testing() {

        // return autoDrive1.withTimeout(2)

        // .andThen(autoDrive2.withTimeout(2))

        // .andThen(Commands.print("END OF DRIVE"))
        // .andThen(Commands.print("isAutoDriveScheduled: " + autoDrive1.isScheduled()))
        // .andThen(Commands.print("isAutoDriveFinished: " + autoDrive1.isFinished()))
        // .andThen(Commands.print("isAutoDriveFinished: " +
        // autoDrive1.getRequirements()))
        // .andThen(Commands.print("isAutoShootFinished: " +
        // autoShoot1.getRequirements()))
        // .andThen(autoShoot1)
        // .andThen(Commands.print("AUTO SHOOT DONE"))
        // .withName("TESTING");

        addCommands(
                // autoDrive1.finallyDo(interrupted -> {
                // if (interrupted) {
                // System.out.println("Auto drive 1 was interrupted!");
                // } else {
                // System.out.println("Auto drive 1 finished!");
                // }
                // }).withTimeout(2),
                // autoDrive2.finallyDo(interrupted -> {
                // if (interrupted) {
                // System.out.println("Auto drive 2 was interrupted!");
                // } else {
                // System.out.println("Auto drive 2 finished!");
                // }
                // }).withTimeout(2),
                autoDrive1.withTimeout(2),
                // Commands.runOnce(() -> {
                // autoDrive1.cancel();
                // }),
                Commands.print("END OF DRIVE"),
                // Commands.print("isAutoDriveScheduled: " + autoDrive1.isScheduled()),
                // Commands.print("isAutoDriveFinished: " + autoDrive1.isFinished()),
                // Commands.print("isAutoDriveRequirements: " + autoDrive1.getRequirements()),
                // Commands.print("isAutoShootRequirements: " + autoShoot1.getRequirements()),
                Commands.print("isAutoIntakeRequirements: " + autoIntake.getRequirements()),

                Commands.runOnce(() -> {
                    System.out.println("TESTING REQUIREMENTS: " + getRequirements());
                }),
                // autoShoot1,
                autoIntake.withTimeout(3),
                Commands.print("AUTO INTAKE DONE"));

        // return autoDrive1.withTimeout(2)
        // .andThen(autoShoot1)
        // .andThen(Commands.print("AUTO SHOOT DONE"))
        // .withName("TESTING");

    }

}
