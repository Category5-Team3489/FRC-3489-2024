// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.LedConstants;

public class LEDs extends SubsystemBase {

    private static final LEDs instance = new LEDs();

    public static LEDs get() {
        return instance;
    }

    private boolean haveTeleopLedsFlashedThisEnable = false;

    PWMSparkMax rightLeds = new PWMSparkMax(LedConstants.RightPort);
    PWMSparkMax leftLeds = new PWMSparkMax(LedConstants.LeftPort);
    public LedState ledState = LedState.Off;

    public enum LedState {
        Off,
        TeleopBlink,
        NeedNote,
        Intaked,
        Outtake,
        ShootNote,
        DarkRed,
        Red,
    }

    public void setLeds(LedState ledState) {
        this.ledState = ledState;

        switch (ledState) {
            case Off: // Off
                stopLeds();
                break;
            case TeleopBlink: // Lawn Green
                setSolidColor(0.71);
                break;
            case NeedNote: // Yellow
                setSolidColor(0.69);
                break;
            case ShootNote: // Blue Violet
                setSolidColor(0.89);
                break;
            case Intaked:
                setSolidColor(0.420);
                break;
            case DarkRed:
                setSolidColor(59);
                break;
            case Outtake: //Ocean
                setSolidColor(-0.95);
                break;
            case Red:
                setSolidColor(61);
        }
    }

    public void setSolidColor(double colorSpeed) {
        rightLeds.set(colorSpeed);
        leftLeds.set(colorSpeed);
    }

    public void stopLeds() {
        rightLeds.set(0.99);
        leftLeds.set(0.99);
    }

    @Override
    public void periodic() {
        tryFlashTeleopLeds();
    }

    public void tryFlashTeleopLeds() {
        if (DriverStation.isDisabled()) {
            haveTeleopLedsFlashedThisEnable = false;
            return;
        }

        if (!DriverStation.isTeleop() || haveTeleopLedsFlashedThisEnable) {
            return;
        }
        Commands.runOnce(() -> setLeds(LedState.TeleopBlink), this)
                .withTimeout(1)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .schedule();

        // Command command = new FlashLeds(this, LedColor.White, 10, 0.1, 0.1)
        // .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

        haveTeleopLedsFlashedThisEnable = true;
    }

    public Command getSolidColorForSecondsCommand(double color, double seconds, boolean isInterruptible) {
        Runnable start = () -> {
            setSolidColor(color);
        };
        Runnable end = () -> {
            stopLeds();
        };

        InterruptionBehavior interruptBehavior = isInterruptible ? InterruptionBehavior.kCancelSelf
                : InterruptionBehavior.kCancelIncoming;

        return Commands.startEnd(start, end, this)
                .withTimeout(seconds)
                .withInterruptBehavior(interruptBehavior);
    }

    // public CommandBase LedDiognostic() {
    // return getSolidColorForSecondsCommand(LedColor.White, 5, true)
    // .withName("Set Solid Color");
    // }
}
