// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of Nicholas was here
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Intake.IntakeUntilDetection;
import frc.robot.commands.Intake.ManualSetIntake;
import frc.robot.commands.shooter.ShooterIntake;
import frc.robot.enums.ClimberState;
import frc.robot.enums.IntakeState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ShooterSpeed;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // ---------------- IDEAS ----------------
    /**
     * Automatic shooter rotation and speed control when close to target and have a
     * piece
     */

    // ---------------- SUBSYSTEMS ----------------
    private final Climber climber = Climber.get();
    private final Intake intake = Intake.get();
    private final IntakeUntilDetection intakeUntilDetection = IntakeUntilDetection.get();
    private final Index index = Index.get();
    private final ShooterSpeed shooterSpeed = ShooterSpeed.get();

    // ---------------- COMMANDS ----------------
    // Shooter
    private final ShooterIntake shooterIntake = new ShooterIntake();
    private final ManualSetIntake manualSetIntake = new ManualSetIntake(intake, index);

    // ---------------- INPUT DEVICES ----------------
    private final CommandXboxController xbox = new CommandXboxController(OperatorConstants.DriverControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Configure bindings with: - xbox.*().*(Command) - new
     * Trigger(BooleanSupplier).*(Command)
     */
    private void configureBindings() {

        xbox.rightTrigger().and(xbox.y()).whileTrue(manualSetIntake.manualSetIntake());
        xbox.y().and(xbox.leftTrigger()).onTrue(shooterIntake);
        xbox.leftBumper().onTrue(Commands.print("See fieldRelative"));
        xbox.rightBumper().and(xbox.leftBumper()).onTrue(Commands.print("Xbox Bumpers"));
        // dpad center = manual shoot
        // right trigger and y = manual intake (no laser sensor)
        xbox.leftTrigger().and(xbox.y()).onTrue(Commands.print("ManualIntake"));
        xbox.rightTrigger().and(xbox.y()).onTrue(Commands.print("ManualIntake"));
        // right joystick = shooter angle

        // bumpers = set manual shootstate (speed/angle)

        // a = auto shoot
        xbox.a().whileTrue(shooterSpeed.shootNote());
        // x = stop shooter;
        xbox.x().onTrue(shooterSpeed.shooterStop());
        // y = intake
        xbox.y().onTrue(intakeUntilDetection);
        // b = outtake
        xbox.b().onTrue(intake.intakeCommand(IntakeState.Out));

        // Climber
        xbox.povUp().whileTrue(climber.climberCommand(ClimberState.PovUp));
        xbox.povDown().whileTrue(climber.climberCommand(ClimberState.PovDown));
        xbox.povLeft().whileTrue(climber.climberCommand(ClimberState.PovLeft));
        xbox.povRight().whileTrue(climber.climberCommand(ClimberState.PovRight));
        xbox.povDownLeft().whileTrue(climber.climberCommand(ClimberState.PovDownLeft));
        xbox.povDownRight().whileTrue(climber.climberCommand(ClimberState.PovDownRight));
        xbox.povUpLeft().whileTrue(climber.climberCommand(ClimberState.PovUpLeft));
        xbox.povUpRight().whileTrue(climber.climberCommand(ClimberState.PovUpRight));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return Commands.print("TODO Auto");
    }

    /*
     * Intake intake game piece until dectection when detection occurs, stop the
     * belt, the intake, and
     * initiate the falcon motors manual intake/belt until stop button stop intake
     * outtake Game piece
     * move intake and belt
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
