// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of Nicholas was here
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Intake.CoralIntake;
import frc.robot.commands.Intake.IntakeUntilDetectionAngle;
import frc.robot.commands.Intake.Outtake;
import frc.robot.commands.autoShooting.AutoShoot;
import frc.robot.commands.autoShooting.AutoShootTest;
import frc.robot.commands.autos.Cat5Autos;
import frc.robot.commands.autos.LeftShootIntakeAuto;
// import frc.robot.commands.autos.CenterShootIntakeShoot;
import frc.robot.commands.autos.Nothing;
import frc.robot.commands.autos.RightCenterLine2Piece;
import frc.robot.commands.autos.RightCenterLine3Piece;
import frc.robot.commands.autos.RightShootIntakeAuto;
import frc.robot.commands.autos.Shoot;
import frc.robot.commands.autos.ShootIntakeAutoShoot;
import frc.robot.commands.autos.ShootTaxi;
import frc.robot.commands.autos.ThreePieceAuto;
import frc.robot.commands.autos.SideShootIntakeShoot;
import frc.robot.commands.autos.Taxi;
import frc.robot.commands.shooter.SetShooterSpeedAndAngle;
import frc.robot.commands.shooter.SetShooterSpeedAngleDifferent;
import frc.robot.commands.shooter.ShooterIntake2;
import frc.robot.enums.ClimberState;
import frc.robot.enums.IndexState;
import frc.robot.enums.IntakeState;
import frc.robot.enums.ShooterAngleState;
import frc.robot.enums.SpeedLimitState;
import frc.robot.subsystems.AprilLimelight;
// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralLimelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterAngle;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
        private static final double MaxMetersPerSecond = Constants.Drivetrain.MaxMetersPerSecond;
        private static final double MaxRadiansPerSecond = Constants.Drivetrain.MaxRadiansPerSecond;

        private final Command autoShoot = new AutoShoot().onlyWhile(() -> isNotDriving());
        private final Command coralIntake = new CoralIntake().onlyWhile(() -> isNotDriving());

        private final Cat5Autos autos = new Cat5Autos();

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
                addAutos();
        }

        private void configureBindings() {
                bindDriveTrain();
                bindIntakeIndex();
                bindShooter();
        }

        private void bindDriveTrain() {

                final Drivetrain drivetrain = Drivetrain.get();
                final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                                .withDeadband(MaxMetersPerSecond * 0.1)
                                .withRotationalDeadband(MaxRadiansPerSecond * 0.1) // Add a
                                                                                   // 10%
                                                                                   // deadband
                                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                                                         // driving in open loop

                final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
                final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
                final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle();

                driveFacingAngle.HeadingController.setP(8);
                // driveFacingAngle.HeadingController.setD(0.2);
                driveFacingAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

                drivetrain.setDefaultCommand(// Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(-driverXbox.getLeftY() * MaxMetersPerSecond
                                                                * drivetrain.getSpeedLimit()) // Drive
                                                                                              // forward
                                                                                              // with
                                                                                              // negative
                                                                                              // Y
                                                // (forward)
                                                .withVelocityY(-driverXbox.getLeftX() * MaxMetersPerSecond
                                                                * drivetrain.getSpeedLimit()) // Drive
                                                                                              // left
                                                                                              // with
                                                // negative
                                                // X (left)
                                                .withRotationalRate(-driverXbox.getRightX() * MaxRadiansPerSecond
                                                                * drivetrain.getSpeedLimit()) // Drive
                                                                                              // counterclockwise
                                // with negative X (left)
                                ));

                driverXbox.rightTrigger().onTrue(Commands.runOnce(() -> coralIntake.schedule()));

                driverXbox.y().whileTrue(
                                drivetrain.applyRequest(() -> driveFacingAngle
                                                .withVelocityX(-driverXbox.getLeftY() * MaxMetersPerSecond *
                                                                drivetrain.getSpeedLimit())
                                                .withVelocityY(-driverXbox.getLeftX() * MaxMetersPerSecond *
                                                                drivetrain.getSpeedLimit())
                                                .withTargetDirection(Rotation2d.fromDegrees(0))));
                driverXbox.b().whileTrue(
                                drivetrain.applyRequest(() -> driveFacingAngle
                                                .withVelocityX(-driverXbox.getLeftY() * MaxMetersPerSecond *
                                                                drivetrain.getSpeedLimit())

                                                .withVelocityY(-driverXbox.getLeftX() * MaxMetersPerSecond *
                                                                drivetrain.getSpeedLimit())
                                                .withTargetDirection(Rotation2d.fromDegrees(270))));
                driverXbox.a().whileTrue(
                                drivetrain.applyRequest(() -> driveFacingAngle
                                                .withVelocityX(-driverXbox.getLeftY() * MaxMetersPerSecond *
                                                                drivetrain.getSpeedLimit())

                                                .withVelocityY(-driverXbox.getLeftX() * MaxMetersPerSecond *
                                                                drivetrain.getSpeedLimit())
                                                .withTargetDirection(Rotation2d.fromDegrees(180))));
                driverXbox.x().whileTrue(
                                drivetrain.applyRequest(() -> driveFacingAngle
                                                .withVelocityX(-driverXbox.getLeftY() * MaxMetersPerSecond *
                                                                drivetrain.getSpeedLimit())

                                                .withVelocityY(-driverXbox.getLeftX() * MaxMetersPerSecond *
                                                                drivetrain.getSpeedLimit())
                                                .withTargetDirection(Rotation2d.fromDegrees(90))));

                // driverXbox.a().onTrue(Commands.runOnce(() ->
                // testController.setRumble(RumbleType.kBothRumble, 1)));

                // POV slow driving
                // driverXbox.pov(360).whileTrue(
                // // drivetrain.applyRequest(
                // // () -> driveFacingAngle.withVelocityY(0.2 * MaxMetersPerSecond *
                // drivetrain.getSpeedLimit())));
                // Commands.parallel(
                // drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(0,
                // 1))),
                // drivetrain.applyRequest(() -> drive.withVelocityY(0.4))));

                driverXbox.pov(45).whileTrue(
                                drivetrain.applyRequest(
                                                () -> driveFacingAngle
                                                                .withVelocityX(0.2 * MaxMetersPerSecond
                                                                                * drivetrain.getSpeedLimit())
                                                                .withVelocityY(0.2 * MaxMetersPerSecond
                                                                                * drivetrain.getSpeedLimit())));
                driverXbox.pov(90).whileTrue(
                                drivetrain.applyRequest(
                                                () -> driveFacingAngle.withVelocityX(0.2 * MaxMetersPerSecond
                                                                * drivetrain.getSpeedLimit())));
                driverXbox.pov(135).whileTrue(
                                drivetrain.applyRequest(
                                                () -> driveFacingAngle
                                                                .withVelocityX(0.2 * MaxMetersPerSecond
                                                                                * drivetrain.getSpeedLimit())
                                                                .withVelocityY(0.2 * MaxMetersPerSecond
                                                                                * drivetrain.getSpeedLimit())));
                driverXbox.pov(180).whileTrue(
                                drivetrain.applyRequest(
                                                () -> driveFacingAngle.withVelocityY(-0.2 * MaxMetersPerSecond
                                                                * drivetrain.getSpeedLimit())));
                driverXbox.pov(225).whileTrue(
                                drivetrain.applyRequest(
                                                () -> driveFacingAngle
                                                                .withVelocityY(-0.2 * MaxMetersPerSecond
                                                                                * drivetrain.getSpeedLimit())
                                                                .withVelocityX(-0.2 * MaxMetersPerSecond
                                                                                * drivetrain.getSpeedLimit())));
                driverXbox.pov(270).whileTrue(
                                drivetrain.applyRequest(
                                                () -> driveFacingAngle.withVelocityX(-0.2 * MaxMetersPerSecond
                                                                * drivetrain.getSpeedLimit())));
                driverXbox.pov(315).whileTrue(
                                drivetrain.applyRequest(
                                                () -> driveFacingAngle
                                                                .withVelocityY(0.2 * MaxMetersPerSecond
                                                                                * drivetrain.getSpeedLimit())
                                                                .withVelocityX(-0.2 * MaxMetersPerSecond
                                                                                * drivetrain.getSpeedLimit())));

                // Brake
                driverXbox.back().whileTrue(drivetrain.applyRequest(() -> brake));

                // reset the field-centric heading on left bumper press
                driverXbox.start().onTrue(Commands.runOnce(() -> drivetrain.seedFieldRelative()));

                // Speed Buttons- Update the
                driverXbox.leftBumper().onTrue(Commands.runOnce(() -> drivetrain.setSpeedLimit(SpeedLimitState.Full)));
                driverXbox.rightBumper()
                                .onTrue(Commands.runOnce(() -> drivetrain.setSpeedLimit(SpeedLimitState.Forth)));
                // Reset the speed when button is released
                driverXbox.leftBumper().onFalse(Commands.runOnce(() -> drivetrain.setSpeedLimit(SpeedLimitState.Half)));
                driverXbox.rightBumper()
                                .onFalse(Commands.runOnce(() -> drivetrain.setSpeedLimit(SpeedLimitState.Half)));

                if (Utils.isSimulation()) {
                        drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
                }
        }

        private void bindIntakeIndex() {
                final Intake intake = Intake.get();
                final Index index = Index.get();
                final ShooterAngle shooterAngle = ShooterAngle.get();

                final IntakeUntilDetectionAngle intakeUntilDetection = new IntakeUntilDetectionAngle();
                final Outtake outtake = new Outtake();

                Trigger laserTrigger = new Trigger(index.laserSensor::get);

                // tesController.getAButton().onTrue(Commands.runOnce(() ->
                // tesController.setRumble(RumbleType.kLeftRumble, 1), this));

                // TODO Test This with robot
                laserTrigger
                                .debounce(0.29, DebounceType.kRising) // 0.29
                                .onTrue(Commands.runOnce(() -> {
                                        if (intake.hasIntakeBeenSet && !autoShoot.isScheduled()) { // &&
                                                                                                   // index.getMotorSpeed()
                                                                                                   // < 0) {
                                                // intake.hasIntakeBeenSet = false;
                                                // Commands.runOnce(() ->
                                                // driverXbox.getHID().setRumble(RumbleType.kBothRumble, 2));
                                                // Commands.runOnce(() ->
                                                // manipulatorXbox.getHID().setRumble(RumbleType.kBothRumble, 2));

                                                intake.stop();
                                                index.stop();
                                                shooterAngle.updateCommand(ShooterAngleState.Start.getAngle());
                                                System.out.print("Laser Stop");
                                        }
                                }));

                laserTrigger.onTrue(Commands
                                .parallel(Commands.runOnce(
                                                () -> manipulatorXbox.getHID().setRumble(RumbleType.kBothRumble, 0.6)),
                                                Commands.waitSeconds(0.2))
                                .andThen(() -> manipulatorXbox.getHID().setRumble(RumbleType.kBothRumble, 0)));
                laserTrigger.onTrue(Commands.parallel(Commands.runOnce(
                                () -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0.6)),
                                Commands.waitSeconds(0.2))
                                .andThen(() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0)));

                // TODO Remove after testing
                // manipulatorXbox.leftTrigger().onTrue(
                // Commands.parallel(intake.intakeCommand(IntakeState.Out),
                // index.indexCommand(IndexState.Intake)));

                manipulatorXbox.rightTrigger().onTrue(Commands.runOnce(() -> {
                        if (outtake.hasOuttakeBeenSet) {
                                intake.stop();
                                index.stop();
                                outtake.hasOuttakeBeenSet = false;
                                System.out.println("stopMotors--------");
                        } else {
                                outtake.schedule();
                                System.out.println("Outtake Scheduled");
                        }
                }));

                // Trigger test = new Trigger(index.isNoteDetected());

                // index.isNoteDetected().onTrue(index.stop());

                // a = intake/stop intake
                // manipulatorXbox.a().onTrue(Commands.runOnce(() -> {
                // if (intakeUntilDetection.isScheduled()) {
                // intakeUntilDetection.cancel();
                // } else {
                // intakeUntilDetection.schedule();
                // }
                // }));

                manipulatorXbox.a().onTrue(Commands.runOnce(() -> {
                        if (intake.hasIntakeBeenSet) {
                                intake.stop();
                                index.stop();
                                // intake.hasIntakeBeenSet = false;
                                System.out.println("stopMotors--------");
                        } else {
                                // TODO TEST
                                intakeUntilDetection.schedule();

                                System.out.println("Scheduled");
                        }
                }));

        }

        private boolean isNotDriving() {
                return Math.abs(driverXbox.getLeftX()) < 0.1 && Math.abs(driverXbox.getLeftY()) < 0.1
                                && Math.abs(driverXbox.getRightX()) < 0.1;
        }

        private void bindShooter() {
                final ShooterSpeed shooterSpeed = ShooterSpeed.get();
                final ShooterAngle shooterAngle = ShooterAngle.get();
                final Index index = Index.get();
                final ShooterIntake2 shooterIntake2 = new ShooterIntake2();

                final Intake intake = Intake.get();

                final SetShooterSpeedAndAngle setShooterFar = new SetShooterSpeedAndAngle(
                                Constants.ShooterAngle.FarShooterAngle,
                                Constants.ShooterSpeed.DefaultSpeedPercent);
                final SetShooterSpeedAndAngle setShooterClose = new SetShooterSpeedAndAngle(
                                Constants.ShooterAngle.CloseShooterAngle,
                                Constants.ShooterSpeed.CloseShooterSpeed);

                // final SetShooterSpeedAndAngle setShooterAmp = new SetShooterSpeedAndAngle(
                // Constants.ShooterAngle.AmpShooterAngle,
                // );

                final SetShooterSpeedAngleDifferent setShooterAmp = new SetShooterSpeedAngleDifferent(
                                Constants.ShooterAngle.AmpShooterAngle, Constants.ShooterSpeed.AmpBottomShooterSpeed,
                                Constants.ShooterSpeed.AmpTopShooterSpeed); // 40, 30 = amp //60-60 = trap

                // b = stop Shooter
                manipulatorXbox.b().onTrue(shooterSpeed.stopCommand());
                // x = Manual Shoot
                manipulatorXbox.x().onTrue(index.indexCommand(IndexState.Intake));

                // TODO Remove After Testing
                // manipulatorXbox.leftTrigger().onTrue(shooterIntake);

                manipulatorXbox.leftTrigger().onTrue(shooterIntake2);

                // back = Shooter Home Angle
                manipulatorXbox.start().onTrue(shooterAngle.updateCommand(() -> ShooterAngleState.Start.getAngle())
                                .finallyDo(() -> intake.hasIntakeBeenSet = false));

                // TODO remove after testing
                // manipulatorXbox.y().onTrue(shooterSpeed.updateCommand(0.6));

                // Manual Shooter Angle
                manipulatorXbox.axisLessThan(5, -0.1).whileTrue(
                                shooterAngle.adjustManualAngle(1).finallyDo(() -> intake.hasIntakeBeenSet = false));
                manipulatorXbox.axisGreaterThan(5, 0.1).whileTrue(
                                shooterAngle.adjustManualAngle(-1).finallyDo(() -> intake.hasIntakeBeenSet = false));

                // set manual speed/angle
                manipulatorXbox.rightBumper().onTrue(setShooterClose.finallyDo(() -> intake.hasIntakeBeenSet = false));
                // manipulatorXbox.leftBumper().onTrue(setShooterFar.finallyDo(() ->
                // intake.hasIntakeBeenSet = false));
                manipulatorXbox.leftBumper().onTrue(setShooterAmp.finallyDo(() -> intake.hasIntakeBeenSet = false));

                // TODO y = Auto Shoot
                // manipulatorXbox.y().onTrue(autoShoot);

                // manipulatorXbox.y().onTrue(Commands.runOnce(() -> {
                // if (autoShoot.isScheduled()) {
                // autoShoot.cancel();
                // } else {
                // autoShoot.schedule();
                // }
                // }));

                // TODO Test
                manipulatorXbox.y().onTrue(Commands.runOnce(() -> autoShoot.schedule()));

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                return autos.getAutonomousCommand();
        }

        private void addAutos() {
                // Taxi
                autos.addAuto(() -> {
                        Taxi taxiCommand = new Taxi();
                        return taxiCommand.taxi();
                });

                // Nothing
                autos.addAuto(() -> new Nothing()
                                .withName("NothingAuto"));

                // ShootTaxi
                autos.addAuto(() -> {
                        ShootTaxi shootTaxiCommand = new ShootTaxi();
                        return shootTaxiCommand.shootTaxi();
                });

                // Three Piece (center line)
                // TODO Get This Working
                autos.addAuto(() -> {
                        ThreePieceAuto threePiece = new ThreePieceAuto();
                        return threePiece.threePieceAuto();
                });

                // Shoot Intake Auto Shoot
                autos.addAuto(() -> {
                        ShootIntakeAutoShoot shootIntakeAutoShootCommand = new ShootIntakeAutoShoot();
                        return shootIntakeAutoShootCommand.shootIntakeAutoShoot();
                });

                // Shoot
                autos.addAuto(() -> {
                        Shoot shootCommand = new Shoot();
                        return shootCommand.shoot();
                });

                // Right two piece
                // TODO Get this working
                autos.addAuto(() -> {
                        RightShootIntakeAuto rightShootIntakeAutoCommand = new RightShootIntakeAuto();
                        return rightShootIntakeAutoCommand.rightShootIntakeAuto();
                });

                // Left Two Piece
                // TODO Get this working
                autos.addAuto(() -> {
                        LeftShootIntakeAuto leftShootIntakeAutoCommand = new LeftShootIntakeAuto();
                        return leftShootIntakeAutoCommand.leftShootIntakeAuto();
                });

                // Side Shoot Intake Shoot
                autos.addAuto(() -> {
                        SideShootIntakeShoot sideShootIntakeShootCommand = new SideShootIntakeShoot();
                        return sideShootIntakeShootCommand.sideShootIntakeShoot();
                });

                // right center line 3 piece
                autos.addAuto(() -> {
                        RightCenterLine3Piece rightCenterLine3PieceCommand = new RightCenterLine3Piece();
                        return rightCenterLine3PieceCommand.rightCenterLine3Piece();
                });

                // right center line 3 piece
                autos.addAuto(() -> {
                        RightCenterLine2Piece rightCenterLine2PieceCommand = new RightCenterLine2Piece();
                        return rightCenterLine2PieceCommand.rightCenterLine2Piece();
                });

                autos.addSelectorWidget();
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
