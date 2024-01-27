package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Enums.BeltState;
import frc.robot.Enums.ShooterState;
import frc.robot.Enums.BeltState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Belt extends SubsystemBase {

    public Belt() {}



    private final CANSparkMax beltmotor = new CANSparkMax(0, MotorType.kBrushless);
    private final DigitalInput laserSensor = new DigitalInput(0);

    //move belt
    public void moveBelt (BeltState beltEnumSpeed) {
        beltmotor.set(beltEnumSpeed.getbeltEnumSpeed());
    }

    public void moveBeltShooter (BeltState beltEnumSpeed) {
        beltmotor.set(beltEnumSpeed.getbeltEnumSpeed());
    }
    public void moveDoubleBelt(double beltSpeed) {
        beltmotor.set(beltSpeed);
    }

    public boolean isNoteDetected() {
        return laserSensor.get();
    }

    public void stopBelt() {
        beltmotor.stopMotor();
    }
    
    //encoder position change (constant) PID



    //start intake/belt
    //wait to get sensor value (could have this start shooter motor)
    //stop


    //set shooter speed
    //press shoot button
    //run belt/shooter
    //stop





 
}