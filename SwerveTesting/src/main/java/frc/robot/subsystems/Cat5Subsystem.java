package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public abstract class Cat5Subsystem extends SubsystemBase {
    protected final RobotContainer robotContainer;

    protected Cat5Subsystem(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;

        robotContainer.initSubsystem(getName());
    }
}
