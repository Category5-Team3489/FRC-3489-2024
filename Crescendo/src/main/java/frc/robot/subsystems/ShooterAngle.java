package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

// Power robot on with shooter angle in home position
public class ShooterAngle extends SubsystemBase {
    private static ShooterAngle instance = new ShooterAngle();

    public static ShooterAngle get() {
        return instance;
    }

    // Constants
    // TODO SET CAN IDS
    private final int LeftMotorId = 0;
    private final int RightMotorId = 1;

    // Gear ratio
    private static final double MotorRotationsPerRevolution = (100.0 / 1.0) * (2.0 / 1.0);
    // Convert revolutions to degrees (Used in PID)
    private static final double MotorRotationsPerDegree = MotorRotationsPerRevolution / 360.0;
    private static final double DegreesPerMotorRotation = 1.0 / MotorRotationsPerDegree;

    // Shooter is at its target angle if the error
    // is within plus or minus this value
    private static final double AllowedErrorDegrees = 2.0;

    // From the perspective of the robot
    private final CANSparkMax leftMotor = new CANSparkMax(LeftMotorId, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(RightMotorId, MotorType.kBrushless);

    private final SparkPIDController pidController = rightMotor.getPIDController();
    private final RelativeEncoder encoder = rightMotor.getEncoder();

    private double targetAngleDegrees = Double.NaN;

    private ShooterAngle() {
        leftMotor.setInverted(true);
        leftMotor.follow(rightMotor);
    }

    public void setAngle(double angleDegrees) {
        targetAngleDegrees = angleDegrees;

        double targetRotations = angleDegrees * MotorRotationsPerDegree;
        pidController.setReference(targetRotations, ControlType.kPosition, 0);
    }

    public boolean isAtTarget() {
        if (Double.isNaN(targetAngleDegrees)) {
            return false;
        }

        double rotations = encoder.getPosition();
        double angleDegrees = rotations * DegreesPerMotorRotation;

        double errorDegrees = targetAngleDegrees - angleDegrees;
        errorDegrees = Math.abs(errorDegrees);

        return errorDegrees < AllowedErrorDegrees;
    }

    // manual shooting
    // adjust angle
    // adjust speed
    // move belt index

}
