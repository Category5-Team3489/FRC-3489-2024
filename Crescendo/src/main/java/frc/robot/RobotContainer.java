// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of Nicholas was here
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Intake.IntakeUntilDetection;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShooterIntake;
import frc.robot.enums.ClimberState;
import frc.robot.enums.IndexState;
import frc.robot.enums.IntakeState;
import frc.robot.enums.SpeedLimitState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterAngle;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

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
            / Math.hypot(DrivetrainConstants.kFrontLeftXPosInches,
                    DrivetrainConstants.kFrontLeftYPosInches);

    // ---------------- INPUT DEVICES ----------------
    private final CommandXboxController manipulatorXbox = new CommandXboxController(
            OperatorConstants.ManipulatorControllerPort);
    private final CommandXboxController driverXbox = new CommandXboxController(
            OperatorConstants.DriverControllerPort);

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
        bindDriveTrain();
        // bindClimber();
        bindIntakeIndex();
        bindShooter();
    }

    private void bindClimber() {
        final Climber climber = Climber.get();
        manipulatorXbox.povUp().whileTrue(climber.climberCommand(ClimberState.PovUp));
        manipulatorXbox.povDown().whileTrue(climber.climberCommand(ClimberState.PovDown));
        manipulatorXbox.povLeft().whileTrue(climber.climberCommand(ClimberState.PovLeft));
        manipulatorXbox.povRight().whileTrue(climber.climberCommand(ClimberState.PovRight));
        manipulatorXbox.povDownLeft().whileTrue(climber.climberCommand(ClimberState.PovDownLeft));
        manipulatorXbox.povDownRight().whileTrue(climber.climberCommand(ClimberState.PovDownRight));
        manipulatorXbox.povUpLeft().whileTrue(climber.climberCommand(ClimberState.PovUpLeft));
        manipulatorXbox.povUpRight().whileTrue(climber.climberCommand(ClimberState.PovUpRight));
    }

    private void bindDriveTrain() {

        // TODO slow directions

        final Drivetrain drivetrain = Drivetrain.get();
        final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDeadband(MaxMetersPerSecond * 0.1).withRotationalDeadband(MaxRadiansPerSecond * 0.1) // Add a
                                                                                                          // 10%
                                                                                                          // deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                                         // driving in open loop
        final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle();

        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-driverXbox.getLeftY() * MaxMetersPerSecond * drivetrain.getSpeedLimit()) // Drive
                                                                                                                 // forward
                                                                                                                 // with
                                                                                                                 // negative
                                                                                                                 // Y
                        // (forward)
                        .withVelocityY(-driverXbox.getLeftX() * MaxMetersPerSecond * drivetrain.getSpeedLimit()) // Drive
                                                                                                                 // left
                                                                                                                 // with
                        // negative
                        // TODO add speed limit to rotations // X (left)
                        .withRotationalRate(-driverXbox.getRightX() * MaxRadiansPerSecond) // Drive counterclockwise
                                                                                           // with negative X (left)
                ));

        driverXbox.back().whileTrue(drivetrain.applyRequest(() -> brake));
        driverXbox.b().whileTrue(drivetrain
                .applyRequest(() -> point
                        .withModuleDirection(new Rotation2d(-driverXbox.getLeftY(),
                                -driverXbox.getLeftX()))));

        // reset the field-centric heading on left bumper press
        driverXbox.start().onTrue(Commands.runOnce(() -> drivetrain.seedFieldRelative()));

        // TODO Speed Buttons
        // Speed Buttons- Update the
        driverXbox.leftBumper().onTrue(Commands.runOnce(() -> drivetrain.setSpeedLimit(SpeedLimitState.Full)));
        driverXbox.rightBumper().onTrue(Commands.runOnce(() -> drivetrain.setSpeedLimit(SpeedLimitState.Forth)));
        // Reset the speed when button is released
        driverXbox.leftBumper().onFalse(Commands.runOnce(() -> drivetrain.setSpeedLimit(SpeedLimitState.Half)));
        driverXbox.rightBumper().onFalse(Commands.runOnce(() -> drivetrain.setSpeedLimit(SpeedLimitState.Half)));

        // TODO Cardinal Directions
        // Cardinal turns
        driverXbox.y().onTrue(drivetrain.applyRequestOnce(() -> drive.withRotationalRate(MaxRadiansPerSecond * 0)));
        driverXbox.b().onTrue(drivetrain.applyRequestOnce(() -> drive.withRotationalRate(MaxRadiansPerSecond * -90)));
        driverXbox.a().onTrue(drivetrain.applyRequestOnce(() -> drive.withRotationalRate(MaxRadiansPerSecond * -180)));
        driverXbox.x().onTrue(drivetrain.applyRequestOnce(() -> drive.withRotationalRate(MaxRadiansPerSecond * -270)));

        // Slow directions
        // driverXbox.povUp().onTrue(drivetrain.applyRequest(() -> ))

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
    }

    private void bindIntakeIndex() {
        final Intake intake = Intake.get();
        final Index index = Index.get();

        final IntakeUntilDetection intakeUntilDetection = new IntakeUntilDetection();

        // a and right Trigger = outtake
        manipulatorXbox.rightTrigger().and(manipulatorXbox.a()).onTrue(intake.intakeCommand(IntakeState.Out));

        // a = intake/stop intake
        // manipulatorXbox.a().onTrue(Commands.runOnce(() -> {
        // if (intakeUntilDetection.isScheduled()) {
        // intakeUntilDetection.cancel();
        // } else {
        // intakeUntilDetection.schedule();
        // }
        // }));

        manipulatorXbox.a().onTrue(Commands.runOnce(() -> {
            if (intakeUntilDetection.hasIntakeBeenSet) {
                intake.stop();
                index.stop();
                intakeUntilDetection.hasIntakeBeenSet = false;
                System.out.println("----------stopMotors");
            } else {
                intakeUntilDetection.schedule();
                System.out.println("Scheduled");
            }
        }));

    }

    private void bindShooter() {
        final ShooterSpeed shooterSpeed = ShooterSpeed.get();
        final ShooterAngle shooterAngle = ShooterAngle.get();
        final Index index = Index.get();
        final ShooterIntake shooterIntake = new ShooterIntake();

        final Shoot setShooterFar = new Shoot(
                () -> Constants.ShooterAngle.FarShooterAngle,
                () -> Constants.ShooterSpeed.FarShooterSpeed);
        final Shoot setShooterClose = new Shoot(
                () -> Constants.ShooterAngle.CloseShooterAngle,
                () -> Constants.ShooterSpeed.CloseShooterSpeed);

        // b = stop Shooter
        manipulatorXbox.b().onTrue(shooterSpeed.stopCommand());
        // x = Manual Shoot
        manipulatorXbox.x().onTrue(index.indexCommand(IndexState.Outtake));
        // a and left trigger = Shoter Intake
        manipulatorXbox.a().and(manipulatorXbox.leftTrigger()).onTrue(shooterIntake);

        // TODO remove after testing
        manipulatorXbox.y().onTrue(shooterSpeed.setMotorPercent(() -> 1));

        // Manual Shooter Angle
        // TODO Uncomment after testing
        manipulatorXbox.axisLessThan(5, -0.1).whileTrue(shooterAngle.adjustManualAngle(-1));
        manipulatorXbox.axisGreaterThan(5,0.1).whileTrue(shooterAngle.adjustManualAngle(1));

        // set manual speed/angle
        // manipulatorXbox.rightBumper().onTrue(setShooterClose);
        // manipulatorXbox.leftBumper().onTrue(setShooterFar);

        // TODO y = Auto Shoot
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
