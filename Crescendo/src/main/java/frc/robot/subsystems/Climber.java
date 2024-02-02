package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

public class Climber extends SubsystemBase {

    private final TalonFX motor1 = new TalonFX(0);
    private final TalonFX motor2 = new TalonFX(1);

    public Climber() {
        motor2.setInverted(true);
    }

    /**
     * @param speedRps
     */
    public void setSpeed(double speedRps) {
        motor1.set(speedRps);
        motor2.set(speedRps);
    }
}
}
