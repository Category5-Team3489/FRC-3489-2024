package frc.robot.subsystems;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralLimelight extends SubsystemBase {

    private static final CoralLimelight instance = new CoralLimelight();

    public static CoralLimelight get() {
        return instance;
    }

    private Timer timer = new Timer();

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-intake");
    private final NetworkTableEntry tagIdEntry = table.getEntry("tid");
    private final NetworkTableEntry tx = table.getEntry("tx");
    private final NetworkTableEntry ty = table.getEntry("ty");
    private final NetworkTableEntry ta = table.getEntry("ta");
    private final NetworkTableEntry tv = table.getEntry("tv");

    private double lastTargetX;
    private double lastTargetY;

    private CoralLimelight() {

        register();

        try {
            HttpCamera limelightFeed = new HttpCamera("limelight-intake", "http://10.34.89.99:5800/stream.mjg");// http://10.34.89.11:5800/stream.mjpg

            Shuffleboard.getTab("Main")
                    .add(limelightFeed)
                    .withWidget(BuiltInWidgets.kCameraStream);
        } catch (Exception e) {
            System.out.println("Limelight camera had trouble initializing");
        }

        Shuffleboard.getTab("Intake")
                .addDouble("Tag", () -> getTargetVisible())
                .withSize(1, 1)
                .withPosition(7, 2);

        Shuffleboard.getTab("Intake")
                .addDouble("Limelight X", () -> getTargetX())
                .withSize(1, 1)
                .withPosition(5, 2);

        Shuffleboard.getTab("Intake")
                .addDouble("Limelight Y", () -> getTargetY())
                .withSize(1, 1)
                .withPosition(2, 2);

        // read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        // post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

    }

    @Override
    public void periodic() {

    }

    private boolean isTagVisible() {
        return tv.getInteger(0) == 1;
    }

    public long getTagId() {
        return tagIdEntry.getInteger(-1);
    }

    public double getTargetX() {
        if (isTagVisible()) {
            lastTargetX = tx.getDouble(Double.NaN);
        }
        return lastTargetX;
    }

    public double getTargetY() {
        if (isTagVisible()) {
            lastTargetY = ty.getDouble(Double.NaN);
        }
        return lastTargetY;
    }

    public double getTargetArea() {
        return ta.getDouble(Double.NaN);
    }

    public long getTargetVisible() {
        if (isTagVisible()) {
            timer.stop();
            timer.reset();
            return 1;
        }
        timer.start();
        if (timer.hasElapsed(0.5)) {
            return 0;
        }
        return 1;
    }

}
