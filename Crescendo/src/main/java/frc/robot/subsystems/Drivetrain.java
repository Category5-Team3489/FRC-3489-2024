package frc.robot.subsystems;

import java.util.function.Supplier;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.DrivetrainConstants;
import frc.robot.RobotContainer;
import frc.robot.enums.SpeedLimitState;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem {
    // Singleton
    private static final Drivetrain instance = DrivetrainConstants.DriveTrain;

    private Pigeon2 pigeon;
    public SpeedLimitState speedLimit = SpeedLimitState.Half;

    private final double AroundTargetHeadingThresholdDegrees = 5;

    public static Drivetrain construct(SwerveDrivetrainConstants driveTrainConstants,
            SwerveModuleConstants... modules) {
        return new Drivetrain(driveTrainConstants, modules);
    }

    public static Drivetrain get() {
        return instance;
    }

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private Drivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        this.pigeon = new Pigeon2(driveTrainConstants.Pigeon2Id);
    }

    private Drivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        this.pigeon = new Pigeon2(driveTrainConstants.Pigeon2Id);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command applyRequestOnce(Supplier<SwerveRequest> requestSupplier) {
        return runOnce(() -> this.setControl(requestSupplier.get()));
    }

    public boolean isAroundTargetHeading() {
        
        Rotation2d target = Rotation2d.fromDegrees(90);

        return Math.abs(pigeon.getRotation2d().minus(target).getDegrees()) < AroundTargetHeadingThresholdDegrees;
    }

    public double getSpeedLimit() {
        return speedLimit.getSpeedLimit();
    }

    public void setSpeedLimit(SpeedLimitState speedLimitState) {
        speedLimit = speedLimitState;
    }

    // public void povDrive(int degrees) {
    //     if (xMetersPerSecond == 0 && yMetersPerSecond == 0) {
    //         //int degrees = robotContainer.input.getDrivePovAngleDegrees();
    //         if (degrees != -1) {
    //             degrees += 90;

    //             xPercent = Math.sin(Math.toRadians(degrees));
    //             yPercent = Math.cos(Math.toRadians(degrees));

    //             xMetersPerSecond = xPercent * PovSpeedMetersPerSecond;
    //             yMetersPerSecond = yPercent * PovSpeedMetersPerSecond;
    //         }
    //     }
    // }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
