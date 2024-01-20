package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Cat5;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive;
import frc.robot.commands.AutomateLeftDoubleSubstation;
import frc.robot.commands.AutomateRightDoubleSubstation;
import frc.robot.configs.DriveMotorConfig;
import frc.robot.configs.OffsetsConfig;
import frc.robot.data.Cat5DeltaTracker;
import frc.robot.data.shuffleboard.Cat5ShuffleboardLayout;
import frc.robot.data.shuffleboard.Cat5ShuffleboardTab;
import frc.robot.enums.GridPosition;
import frc.robot.enums.LimelightPipeline;
import frc.robot.enums.ModulePosition;

public class Drivetrain extends Cat5Subsystem {
    // Constants
    private static final double AroundTargetHeadingThresholdDegrees = 5;
    public static final double PovSpeedMetersPerSecond = 0.4;
    public static final double FullSpeedRateLimiter100PercentPerSecond = 2.0; // 3.0

    private static final double WheelsLeftToRightMeters = 0.54;
    private static final double WheelsFrontToBackMeters = 0.54;
    private static final double MetersPerRotation = SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

    private static final Translation2d FrontLeftMeters = new Translation2d(WheelsLeftToRightMeters / 2.0, WheelsFrontToBackMeters / 2.0);
    private static final Translation2d FrontRightMeters = new Translation2d(WheelsLeftToRightMeters / 2.0, -WheelsFrontToBackMeters / 2.0);
    private static final Translation2d BackLeftMeters = new Translation2d(-WheelsLeftToRightMeters / 2.0, WheelsFrontToBackMeters / 2.0);
    private static final Translation2d BackRightMeters = new Translation2d(-WheelsLeftToRightMeters / 2.0, -WheelsFrontToBackMeters / 2.0);

    public static final double TheoreticalMaxVelocityMetersPerSecond = 6380.0 / 60.0 * MetersPerRotation;
    public static final double TheoreticalMaxAngularVelocityRadiansPerSecond = TheoreticalMaxVelocityMetersPerSecond / Math.hypot(WheelsLeftToRightMeters / 2.0, WheelsFrontToBackMeters / 2.0);

    public static final SwerveDriveKinematics Kinematics = new SwerveDriveKinematics(
        FrontLeftMeters,
        FrontRightMeters,
        BackLeftMeters,
        BackRightMeters
    );
    
    private static final double OmegaProportionalGainDegreesPerSecondPerDegreeOfError = 8.0;
    private static final double OmegaIntegralGainDegreesPerSecondPerDegreeSecondOfError = 0;
    private static final double OmegaDerivativeGainDegreesPerSecondPerDegreePerSecondOfError = 0.2;
    private static final double OmegaToleranceDegrees = 2.5;
    private static final double OmegaTippyToleranceDegrees = 7.5;
    private static final double OmegaMaxDegreesPerSecond = 720;
    private static final double OmegaFeedforwardDegreesPerSecond = 35;

    private static final double MaxVoltage = 12;

    private static final int FrontLeftDriveDeviceId = 1;
    private static final int FrontLeftSteerDeviceId = 2;
    private static final int FrontLeftEncoderDeviceId = 20;

    private static final int FrontRightDriveDeviceId = 3;
    private static final int FrontRightSteerDeviceId = 4;
    private static final int FrontRightEncoderDeviceId = 21;

    private static final int BackLeftDriveDeviceId = 7;
    private static final int BackLeftSteerDeviceId = 8;
    private static final int BackLeftEncoderDeviceId = 22;

    private static final int BackRightDriveDeviceId = 5;
    private static final int BackRightSteerDeviceId = 6;
    private static final int BackRightEncoderDeviceId = 23;

    // Configs
    public final OffsetsConfig offsetsConfig;

    // Devices
    public final SwerveModule frontLeftModule;
    public final SwerveModule frontRightModule;
    public final SwerveModule backLeftModule;
    public final SwerveModule backRightModule;

    // Suppliers
    private final BooleanSupplier isCameraDisabledWhenDriving;

    // Commands
    private final Drive driveCommand;
    private final AutomateLeftDoubleSubstation automateLeftDoubleSubstationCommand;
    private final AutomateRightDoubleSubstation automateRightDoubleSubstationCommand;
    
    // State
    private final Arm arm;
    private final NavX2 navx;
    private final Limelight limelight;
    private Rotation2d targetHeading = null;
    private final PIDController omegaController = new PIDController(OmegaProportionalGainDegreesPerSecondPerDegreeOfError, OmegaIntegralGainDegreesPerSecondPerDegreeSecondOfError, OmegaDerivativeGainDegreesPerSecondPerDegreePerSecondOfError);

    public Drivetrain(RobotContainer robotContainer, Arm arm, NavX2 navx, Limelight limelight) {
        super(robotContainer);
        this.arm = arm;
        this.navx = navx;
        this.limelight = limelight;

        driveCommand = new Drive(robotContainer, this);
        automateLeftDoubleSubstationCommand = new AutomateLeftDoubleSubstation(arm, limelight, this);
        automateRightDoubleSubstationCommand = new AutomateRightDoubleSubstation(arm, limelight, this);

        setDefaultCommand(driveCommand);

        offsetsConfig = new OffsetsConfig(robotContainer, this);
        
        omegaController.enableContinuousInput(-180.0, 180.0);
        omegaController.setTolerance(OmegaToleranceDegrees);

        if (Constants.IsSwerveDebugEnabled) {
            ShuffleboardTab layout = Cat5ShuffleboardTab.Swerve_Debug.get();

            frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                layout.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
                Mk4SwerveModuleHelper.GearRatio.L2,
                FrontLeftDriveDeviceId,
                FrontLeftSteerDeviceId,
                FrontLeftEncoderDeviceId,
                0.0
            );
    
            frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                layout.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
                Mk4SwerveModuleHelper.GearRatio.L2,
                FrontRightDriveDeviceId,
                FrontRightSteerDeviceId,
                FrontRightEncoderDeviceId,
                0.0
            );
    
            backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                layout.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
                Mk4SwerveModuleHelper.GearRatio.L2,
                BackLeftDriveDeviceId,
                BackLeftSteerDeviceId,
                BackLeftEncoderDeviceId,
                0.0
            );
    
            backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                layout.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
                Mk4SwerveModuleHelper.GearRatio.L2,
                BackRightDriveDeviceId,
                BackRightSteerDeviceId,
                BackRightEncoderDeviceId,
                0.0
            );
        }
        else {
            frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                FrontLeftDriveDeviceId,
                FrontLeftSteerDeviceId,
                FrontLeftEncoderDeviceId,
                0
            );
    
            frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                FrontRightDriveDeviceId,
                FrontRightSteerDeviceId,
                FrontRightEncoderDeviceId,
                0
            );
    
            backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                BackLeftDriveDeviceId,
                BackLeftSteerDeviceId,
                BackLeftEncoderDeviceId,
                0
            );
    
            backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                BackRightDriveDeviceId,
                BackRightSteerDeviceId,
                BackRightEncoderDeviceId,
                0
            );
        }

        GenericEntry isCameraDisabledWhenDrivingEntry = robotContainer.layouts.get(Cat5ShuffleboardLayout.Driver)
            .add("Disable Camera When Driving", false)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();
        isCameraDisabledWhenDriving = () -> isCameraDisabledWhenDrivingEntry.getBoolean(false);

        if (Constants.IsShuffleboardDebugEnabled) {
            {
                var layout = robotContainer.layouts.get(Cat5ShuffleboardLayout.Debug_Drive_Velocities);
                layout.addDouble("Front Left (m per s)", () -> frontLeftModule.getDriveVelocity());
                layout.addDouble("Front Right (m per s)", () -> frontRightModule.getDriveVelocity());
                layout.addDouble("Back Left (m per s)", () -> backLeftModule.getDriveVelocity());
                layout.addDouble("Back Right (m per s)", () -> backRightModule.getDriveVelocity());
                layout.addDouble("Average (m per s)", () -> getAverageDriveVelocityMetersPerSecond());
            }
            {
                var layout = robotContainer.layouts.get(Cat5ShuffleboardLayout.Debug_Drive_Stator_Current);
                layout.addDouble("Front Left (A)", () -> DriveMotorConfig.getDriveMotor(ModulePosition.FrontLeft).getStatorCurrent());
                layout.addDouble("Front Right (A)", () -> DriveMotorConfig.getDriveMotor(ModulePosition.FrontRight).getStatorCurrent());
                layout.addDouble("Back Left (A)", () -> DriveMotorConfig.getDriveMotor(ModulePosition.BackLeft).getStatorCurrent());
                layout.addDouble("Back Right (A)", () -> DriveMotorConfig.getDriveMotor(ModulePosition.BackRight).getStatorCurrent());    
            }
        }

        new Cat5DeltaTracker<Command>(robotContainer, getCurrentCommand(),
        last -> {
            return last != getCurrentCommand();
        }, last -> {
            String lastString = last == null ? "None" : last.getName();
            Command currentCommand = getCurrentCommand();
            String currentString = currentCommand == null ? "None" : currentCommand.getName();
            Cat5.print("Drivetrain current command: " + lastString + " -> " + currentString);
            return currentCommand;
        });
    }

    @Override
    public void periodic() {
        if (arm.getGridPosition() == GridPosition.Mid || arm.getGridPosition() == GridPosition.High) {
            omegaController.setTolerance(OmegaTippyToleranceDegrees);
        }
        else {
            omegaController.setTolerance(OmegaToleranceDegrees);
        }

        if (!DriverStation.isTeleopEnabled()) {
            return;
        }

        if (robotContainer.input.isBeingDriven()) {
            if (!isCameraDisabledWhenDriving.getAsBoolean()) {
                limelight.setDesiredPipeline(LimelightPipeline.Camera);
            }

            if (!isDriveCommandActive()) {
                driveCommand.schedule();
    
                Cat5.print("Drove out of non-drive command during teleop");
            }
        }
        else {
            if (robotContainer.input.shouldAutomateLeftDoubleSubstation()) {
                automateLeftDoubleSubstationCommand.schedule();
            }
            else {
                automateLeftDoubleSubstationCommand.cancel();   
            }

            if (robotContainer.input.shouldAutomateRightDoubleSubstation()) {
                automateRightDoubleSubstationCommand.schedule();
            }
            else {
                automateRightDoubleSubstationCommand.cancel();   
            }
        }
    }

    public boolean isDriveCommandActive() {
        return getCurrentCommand() == driveCommand;
    }

    // targetHeading: increase - CCW
    public void setTargetHeading(Rotation2d targetHeading) {
        this.targetHeading = targetHeading;
    }

    public void resetTargetHeading() {
        targetHeading = null;
        omegaController.reset();
    }

    public boolean isAroundTargetHeading() {
        if (targetHeading == null) {
            return false;
        }

        return Math.abs(navx.getRotation().minus(targetHeading).getDegrees()) < AroundTargetHeadingThresholdDegrees;
    }

    public void brakeTranslation() {
        setFrontLeftPercentAngle(0, Math.toRadians(45 + 90));
        setFrontRightPercentAngle(0, Math.toRadians(45));
        setBackLeftPercentAngle(0, Math.toRadians(45));
        setBackRightPercentAngle(0, Math.toRadians(45 + 90));
    }

    public void brakeRotation() {
        setFrontLeftPercentAngle(0, Math.toRadians(45));
        setFrontRightPercentAngle(0, Math.toRadians(45 + 90));
        setBackLeftPercentAngle(0, Math.toRadians(45 + 90));
        setBackRightPercentAngle(0, Math.toRadians(45));
    }

    // percent: postive forward, angleDegrees zero forward
    public void drivePercentAngle(double percent, double angleDegrees) {
        double angleRadians = Math.toRadians(angleDegrees);
        setFrontLeftPercentAngle(percent, angleRadians);
        setFrontRightPercentAngle(percent, angleRadians);
        setBackLeftPercentAngle(percent, angleRadians);
        setBackRightPercentAngle(percent, angleRadians);
    }

    public void driveFieldRelative(double xMetersPerSecond, double yMetersPerSecond, double speedLimiterPercent, Rotation2d targetHeadingOverride, Double omegaMaxDegreesPerSecondOverride) {
        Rotation2d theta = navx.getRotation();

        if (targetHeading == null) {
            resetTargetHeading();
            targetHeading = theta;
        }
        
        if (targetHeadingOverride != null) {
            targetHeading = targetHeadingOverride;
        }

        double outputDegreesPerSecond = omegaController.calculate(theta.getDegrees(), targetHeading.getDegrees());
        double omegaMaxDegreesPerSecond = OmegaMaxDegreesPerSecond;
        if (omegaMaxDegreesPerSecondOverride != null) {
            omegaMaxDegreesPerSecond = omegaMaxDegreesPerSecondOverride;
        }
        outputDegreesPerSecond = MathUtil.clamp(outputDegreesPerSecond, -omegaMaxDegreesPerSecond, omegaMaxDegreesPerSecond);

        double omegaRadiansPerSecond = 0;
        if (!omegaController.atSetpoint()) {
            outputDegreesPerSecond += Cat5.getSign(outputDegreesPerSecond) * OmegaFeedforwardDegreesPerSecond;
        }

        omegaRadiansPerSecond = Math.toRadians(outputDegreesPerSecond);

        driveFieldRelative(xMetersPerSecond, yMetersPerSecond, omegaRadiansPerSecond, speedLimiterPercent, true);
    }
    
    public void driveFieldRelative(double xMetersPerSecond, double yMetersPerSecond, double omegaRadiansPerSecond, double speedLimiterPercent, boolean isKeepingHeading) {
        if (!isKeepingHeading) {
            resetTargetHeading();
        }
        
        Rotation2d theta = navx.getRotation();

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSecond, yMetersPerSecond, omegaRadiansPerSecond, theta);
    
        SwerveModuleState[] states = Kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, TheoreticalMaxVelocityMetersPerSecond * speedLimiterPercent);

        double frontLeftSteerAngleRadians = states[0].angle.getRadians();
        double frontRightSteerAngleRadians = states[1].angle.getRadians();
        double backLeftSteerAngleRadians = states[2].angle.getRadians();
        double backRightSteerAngleRadians = states[3].angle.getRadians();
        
        setFrontLeftSpeedAngle(states[0].speedMetersPerSecond, frontLeftSteerAngleRadians);
        setFrontRightSpeedAngle(states[1].speedMetersPerSecond, frontRightSteerAngleRadians);
        setBackLeftSpeedAngle(states[2].speedMetersPerSecond, backLeftSteerAngleRadians);
        setBackRightSpeedAngle(states[3].speedMetersPerSecond, backRightSteerAngleRadians);
    }

    // Front Left
    public void setFrontLeftPercentAngle(double percent, double radians) {
        setFrontLeftVoltageAngle(percent * MaxVoltage, radians);
    }
    public void setFrontLeftSpeedAngle(double speedMetersPerSecond, double radians) {
        setFrontLeftVoltageAngle((speedMetersPerSecond / TheoreticalMaxVelocityMetersPerSecond) * MaxVoltage, radians);
    }
    public void setFrontLeftVoltageAngle(double voltage, double radians) {
        frontLeftModule.set(voltage, Cat5.wrapAngle(radians + offsetsConfig.getFrontLeftOffsetRadians()));
    }

    // Front Right
    public void setFrontRightPercentAngle(double percent, double radians) {
        setFrontRightVoltageAngle(percent * MaxVoltage, radians);
    }
    public void setFrontRightSpeedAngle(double speedMetersPerSecond, double radians) {
        setFrontRightVoltageAngle((speedMetersPerSecond / TheoreticalMaxVelocityMetersPerSecond) * MaxVoltage, radians);
    }
    public void setFrontRightVoltageAngle(double voltage, double radians) {
        frontRightModule.set(voltage, Cat5.wrapAngle(radians + offsetsConfig.getFrontRightOffsetRadians()));
    }

    // Back Left
    public void setBackLeftPercentAngle(double percent, double radians) {
        setBackLeftVoltageAngle(percent * MaxVoltage, radians);
    }
    public void setBackLeftSpeedAngle(double speedMetersPerSecond, double radians) {
        setBackLeftVoltageAngle((speedMetersPerSecond / TheoreticalMaxVelocityMetersPerSecond) * MaxVoltage, radians);
    }
    public void setBackLeftVoltageAngle(double voltage, double radians) {
        backLeftModule.set(voltage, Cat5.wrapAngle(radians + offsetsConfig.getBackLeftOffsetRadians()));
    }

    // Back Right
    public void setBackRightPercentAngle(double percent, double radians) {
        setBackRightVoltageAngle(percent * MaxVoltage, radians);
    }
    public void setBackRightSpeedAngle(double speedMetersPerSecond, double radians) {
        setBackRightVoltageAngle((speedMetersPerSecond / TheoreticalMaxVelocityMetersPerSecond) * MaxVoltage, radians);
    }
    public void setBackRightVoltageAngle(double voltage, double radians) {
        backRightModule.set(voltage, Cat5.wrapAngle(radians + offsetsConfig.getBackRightOffsetRadians()));
    }

    public SwerveModulePosition[] getModulePositions() {
        // Distance Meters
        TalonFX frontLeftMotor = DriveMotorConfig.getDriveMotor(ModulePosition.FrontLeft);
        TalonFX frontRightMotor = DriveMotorConfig.getDriveMotor(ModulePosition.FrontRight);
        TalonFX backLeftMotor = DriveMotorConfig.getDriveMotor(ModulePosition.BackLeft);
        TalonFX backRightMotor = DriveMotorConfig.getDriveMotor(ModulePosition.BackRight);

        double frontLeftDistanceMeters = (frontLeftMotor.getSelectedSensorPosition() / 2048.0) * MetersPerRotation;
        double frontRightDistanceMeters = (frontRightMotor.getSelectedSensorPosition() / 2048.0) * MetersPerRotation;
        double backLeftDistanceMeters = (backLeftMotor.getSelectedSensorPosition() / 2048.0) * MetersPerRotation;
        double backRightDistanceMeters = (backRightMotor.getSelectedSensorPosition() / 2048.0) * MetersPerRotation;

        // Rotation
        Rotation2d frontLeftRotation = Rotation2d.fromRadians(Cat5.wrapAngle(frontLeftModule.getSteerAngle() - offsetsConfig.getFrontLeftOffsetRadians()));
        Rotation2d frontRightRotation = Rotation2d.fromRadians(Cat5.wrapAngle(frontRightModule.getSteerAngle() - offsetsConfig.getFrontRightOffsetRadians()));
        Rotation2d backLeftRotation = Rotation2d.fromRadians(Cat5.wrapAngle(backLeftModule.getSteerAngle() - offsetsConfig.getBackLeftOffsetRadians()));
        Rotation2d backRightRotation = Rotation2d.fromRadians(Cat5.wrapAngle(backRightModule.getSteerAngle() - offsetsConfig.getBackRightOffsetRadians()));

        return new SwerveModulePosition[] {
            new SwerveModulePosition(frontLeftDistanceMeters, frontLeftRotation),
            new SwerveModulePosition(frontRightDistanceMeters, frontRightRotation),
            new SwerveModulePosition(backLeftDistanceMeters, backLeftRotation),
            new SwerveModulePosition(backRightDistanceMeters, backRightRotation)
        };
    }

    public double getAverageDriveVelocityMetersPerSecond() {
        return (Math.abs(frontLeftModule.getDriveVelocity()) + Math.abs(frontRightModule.getDriveVelocity()) + Math.abs(backLeftModule.getDriveVelocity()) + Math.abs(backRightModule.getDriveVelocity())) / 4.0;
    }
}
