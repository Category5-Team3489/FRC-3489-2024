package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.IndexState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Index extends SubsystemBase {
    private static Index instance = new Index();

    public static Index get() {
        return instance;
    }

    // Constants
    private static final int MotorId = 15;
    private static final int LaserSensorChannel = 0;

    // NEO 550
    private final CANSparkMax motor = new CANSparkMax(MotorId, MotorType.kBrushless);
    // Diffuse laser sensor
    private final DigitalInput laserSensor = new DigitalInput(LaserSensorChannel);
    // private final RelativeEncoder encoder = motor.getEncoder();

    // move belt

    public Command indexCommand(IndexState state) {
        return Commands.run(() -> {
            System.out.println("index");
            motor.set(state.getIndexEnumSpeed());
        }, this);
    }

    public void stop() {
        motor.stopMotor();
    }

    public boolean isNoteDetected() {
        return laserSensor.get();
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