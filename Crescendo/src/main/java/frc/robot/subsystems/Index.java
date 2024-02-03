package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.IndexState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Index extends SubsystemBase {

    private final CANSparkMax indexMotor = new CANSparkMax(0, MotorType.kBrushless);
    private final DigitalInput laserSensor = new DigitalInput(0);
    private static Index instance = new Index();

    public static Index get() {
        return instance;
    }

    private final RelativeEncoder encoder = indexMotor.getEncoder();

    // move belt
    public void moveIndex(IndexState IndexEnumSpeed) {
        indexMotor.set(IndexEnumSpeed.getIndexEnumSpeed());
    }

    public void moveIndexShooter(IndexState indexEnumSpeed) {
        indexMotor.set(indexEnumSpeed.getIndexEnumSpeed());
    }

    public void moveDoubleIndex(double beltSpeed) {
        indexMotor.set(beltSpeed);
    }

    public boolean isNoteDetected() {
        return laserSensor.get();
    }

    public void stopIndex() {
        indexMotor.stopMotor();
    }

    public double isAtSpeed(double speed) {
        return encoder.getVelocity();
    }

    // encoder position change (constant) PID

    // start intake/belt
    // wait to get sensor value (could have this start shooter motor)
    // stop

    // set shooter speed
    // press shoot button
    // run belt/shooter
    // stop
}