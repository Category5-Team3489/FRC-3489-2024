package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.IntakeState;

public class Intake extends SubsystemBase {
    private static final Intake instance = new Intake();

    public IntakeState intakeState = IntakeState.Off;

    public static Intake get() {
        return instance;
    }

    // Devices
    private final CANSparkFlex centerMotor;
    private final TalonFX rightMotor;
    private final TalonFX leftMotor;

    public boolean hasIntakeBeenSet = false;

    private Intake() {
        //motor declaration
        centerMotor = new CANSparkFlex(11, MotorType.kBrushless);
        rightMotor = new TalonFX(16);
        leftMotor = new TalonFX(15);

        //Shuffleboard
        Shuffleboard.getTab("Main")
                .addString("intake state", () -> intakeState.toString())
                .withSize(1, 1)
                .withPosition(7, 0);

        Shuffleboard.getTab("Main")
                .addDouble("intake speed", () -> centerMotor.getAppliedOutput())
                .withSize(1, 1)
                .withPosition(8, 0);
    }

    //Set intake based on intake state
    public Command intakeCommand(IntakeState centerState, IntakeState falconState) {

        return Commands.runOnce(() -> {
            System.out.println("Center Intake Speed: " + centerState.getSpeed());
            intakeState = centerState;
            hasIntakeBeenSet = true;
            centerMotor.set(centerState.getSpeed());
            rightMotor.set(falconState.getSpeed());
            leftMotor.set(-falconState.getSpeed());
        }, this);
    }

    // Stop Motors
    public void stop() {
        System.out.println("Stop Intake");
        centerMotor.stopMotor();
        rightMotor.stopMotor();
        leftMotor.stopMotor();
        hasIntakeBeenSet = false;
    }
}
