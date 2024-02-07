// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of Nicholas was here
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.shooter.ShooterIntake;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // ---------------- IDEAS ----------------
    /**
     * Automatic shooter rotation and speed control when close to target and have a piece
     */

    // ---------------- SUBSYSTEMS ----------------
    private final Climber climber = Climber.get();

    // ---------------- COMMANDS ----------------
    // Shooter
    private final ShooterIntake shooterIntake = new ShooterIntake();

    // ---------------- INPUT DEVICES ----------------
    private final CommandXboxController xbox =
            new CommandXboxController(OperatorConstants.DriverControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Configure bindings with: - xbox.*().*(Command) - new Trigger(BooleanSupplier).*(Command)
     */
    private void configureBindings() {
        // Climber
        xbox.back().whileTrue(climber.descendCommand());
        xbox.start().whileTrue(climber.ascendCommand());

        // Shooter
        xbox.leftTrigger().and(xbox.y()).onTrue(shooterIntake);

        // xbox.rightTrigger().and(xbox.y()).whileTrue(manualSetIntake.manualSetIntake());
        // xbox.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // xbox.b().whileTrue(drivetrain.applyRequest(
        // () -> point.withModuleDirection(new Rotation2d(-xbox.getLeftY(), -xbox.getLeftX()))));
        // xbox.button(1).onTrue(autoShoot);
        // // reset the field-centric heading on left bumper press
        // xbox.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
        // dpad center = manual shoot
        // left joystick = climber adjustment (already done with the "back" and "start" buttons)
        // left trigger and y = shooter intake
        // right trigger and y = manual intake (no laser sensor)
        // right joystick = shooter angle
        // a = auto shoot
        // x = stop shooter
        // y = intake
        // b = outtake
        // start = reset climber
        // mode = set climber to lowest chain position
        // back = set climber to highest chain position
        // bumpers = set manual shootstate (speed/angle)
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Commands.print("TODO Auto");
    }

    /**
     * Intake intake game piece until dectection when detection occurs, stop the belt, the intake,
     * and initiate the falcon motors manual intake/belt until stop button stop intake outtake Game
     * piece move intake and belt
     */

    // Belt
    // *manual belt
    // *manual belt stop
    // *move belt for shooting (once shooter is at right speed)
    // shooter intake

    // Shooter Angle
    // adjust to set angles (for set distances)
    // reset to home
    // adjust to shortest angle to pass under stage?
    // manually adjust angle
    // auto adjust angle

    // Shooter
    // intake with shooter
    // adjust to set speed
    // auto adjust to speed?
    // stop shooter

    // LEDs
    // in "reved up" state
    // Errors
    // laser sensor out
    // When note has been detected
    // default colors
    // after game piece has been shot

    // Climber
    // Move up and down
    // lock?

    // Hello Brave Traveller
}
