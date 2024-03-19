package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.enums.IndexState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Index extends SubsystemBase {
    private static Index instance = new Index();

    public static Index get() {
        return instance;
    }

    // Constants
    private static final int MotorId = 14;
    private static final int LaserSensorChannel = 9;

    // NEO 550
    private final CANSparkMax motor;
    public final DigitalInput laserSensor;

    private Index() {
        motor = new CANSparkMax(MotorId, MotorType.kBrushless);
        laserSensor = new DigitalInput(LaserSensorChannel);

    }

    public double getMotorSpeed() {
        return motor.get();
    }

    @Override
    public void periodic() {
        // System.out.println("Laser Sensor------- " + laserSensor.get());
        // System.out.println("Note state" + isNoteDetected());
    }

    public Command indexCommand(IndexState state) {
        return Commands.runOnce(() -> {
            System.out.println("======================index");
            motor.set(state.getSpeedPercent());

        }, this);
    }

    public void stop() {
        motor.stopMotor();
    }

    public boolean isNoteDetected() {
        return !laserSensor.get();
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