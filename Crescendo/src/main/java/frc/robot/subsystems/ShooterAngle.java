package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

//Power robot on with shooter angle in home position
public class ShooterAngle extends SubsystemBase {

    // Constants
    private static final double MotorRevolutionsPerRevolution = (100.0 / 1.0) * (2.0 / 1.0);    //gear ratio (revolution)
    private static final double MotorRevolutionsPerDegree = MotorRevolutionsPerRevolution / 360.0;  //convert revolutions to degrees
    private static final double DegreesPerMotorRevolution = 1.0 / MotorRevolutionsPerDegree;    //used in pid

    private final CANSparkMax angleMotorLeft = new CANSparkMax(0, MotorType.kBrushless);
    private final CANSparkMax angleMotorRight = new CANSparkMax (1, MotorType.kBrushless);


    private final SparkPIDController pidController;
    //private final RelativeEncoder encoder;

    public ShooterAngle() {
        angleMotorLeft.follow(angleMotorRight);

        pidController = angleMotorRight.getPIDController();
        //encoder = angleMotorRight.getEncoder();
    }

    public void adjustAngle(double angle) { //angle = degrees
        pidController.setReference(-angle * MotorRevolutionsPerDegree, ControlType.kPosition, 0);
    }

    







 
}