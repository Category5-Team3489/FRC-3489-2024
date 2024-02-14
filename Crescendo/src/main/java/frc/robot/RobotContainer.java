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
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

    // https://www.swervedrivespecialties.com/products/mk4-swerve-module
    private static final double MaxMetersPerSecond = 16.5 / 3.281; // (16.5 ft/s) / (3.281 ft/meter)
    // private static final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a
    // rotation per second max angular velocity
    private static final double MaxRadiansPerSecond = MaxMetersPerSecond
            / Math.hypot(DrivetrainConstants.kFrontLeftXPosInches, DrivetrainConstants.kFrontLeftYPosInches);

    // ---------------- SUBSYSTEMS ----------------
    private final Climber climber = Climber.get();
    private final Intake intake = Intake.get();
    private final IntakeUntilDetection intakeUntilDetection = IntakeUntilDetection.get();
    private final Index index = Index.get();
    private final ShooterSpeed shooterSpeed = ShooterSpeed.get();
    private final Drivetrain drivetrain = Drivetrain.get();

    // ---------------- COMMANDS ----------------
    // Shooter
    private final ShooterIntake shooterIntake = new ShooterIntake();
    private final ManualSetIntake manualSetIntake = new ManualSetIntake(intake, index);

    // Drivetrain
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxMetersPerSecond * 0.1).withRotationalDeadband(MaxRadiansPerSecond * 0.1) // Add a 10%
                                                                                                      // deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                                     // driving in open loop
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle();

    // ---------------- INPUT DEVICES ----------------
    private final CommandXboxController manipulatorXbox = new CommandXboxController(
            OperatorConstants.DriverControllerPort);
    private final CommandXboxController driverXbox = new CommandXboxController(
            OperatorConstants.ManipulatorControllerPort);

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
        // Drivetrain
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driverXbox.getLeftY() * MaxMetersPerSecond) // Drive
                                                                                                               // forward
                                                                                                               // with
                        // negative Y (forward)
                        .withVelocityY(-driverXbox.getLeftX() * MaxMetersPerSecond) // Drive left with negative X (left)
                        .withRotationalRate(-driverXbox.getRightX() * MaxRadiansPerSecond) // Drive counterclockwise
                                                                                           // with negative X (left)
                ));

        driverXbox.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverXbox.b().whileTrue(drivetrain
                .applyRequest(() -> point
                        .withModuleDirection(new Rotation2d(-driverXbox.getLeftY(), -driverXbox.getLeftX()))));

        // reset the field-centric heading on left bumper press
        driverXbox.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        // Intake
        manipulatorXbox.rightTrigger().and(manipulatorXbox.y()).whileTrue(manualSetIntake.manualSetIntake());
        manipulatorXbox.y().and(manipulatorXbox.leftTrigger()).onTrue(shooterIntake);

        manipulatorXbox.leftBumper().onTrue(Commands.print("See fieldRelative"));
        manipulatorXbox.rightBumper().and(manipulatorXbox.leftBumper()).onTrue(Commands.print("Xbox Bumpers"));
        // dpad center = manual shoot
        // right trigger and y = manual intake (no laser sensor)
        manipulatorXbox.leftTrigger().and(manipulatorXbox.y()).onTrue(Commands.print("ManualIntake"));
        manipulatorXbox.rightTrigger().and(manipulatorXbox.y()).onTrue(Commands.print("ManualIntake"));

        // right joystick = shooter angle

        // bumpers = set manual shootstate (speed/angle)

        // a = auto shoot
        manipulatorXbox.a().whileTrue(shooterSpeed.shootNote());
        // x = stop shooter;
        manipulatorXbox.x().onTrue(shooterSpeed.shooterStop());
        // y = intake
        manipulatorXbox.y().onTrue(intakeUntilDetection);
        // b = outtake
        manipulatorXbox.b().onTrue(intake.intakeCommand(IntakeState.Out));

        // Climber
        manipulatorXbox.povUp().whileTrue(climber.climberCommand(ClimberState.PovUp));
        manipulatorXbox.povDown().whileTrue(climber.climberCommand(ClimberState.PovDown));
        manipulatorXbox.povLeft().whileTrue(climber.climberCommand(ClimberState.PovLeft));
        manipulatorXbox.povRight().whileTrue(climber.climberCommand(ClimberState.PovRight));
        manipulatorXbox.povDownLeft().whileTrue(climber.climberCommand(ClimberState.PovDownLeft));
        manipulatorXbox.povDownRight().whileTrue(climber.climberCommand(ClimberState.PovDownRight));
        manipulatorXbox.povUpLeft().whileTrue(climber.climberCommand(ClimberState.PovUpLeft));
        manipulatorXbox.povUpRight().whileTrue(climber.climberCommand(ClimberState.PovUpRight));
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
