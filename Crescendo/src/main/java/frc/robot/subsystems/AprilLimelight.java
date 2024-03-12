package frc.robot.subsystems;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.enums.LimelightPipeline;

public class AprilLimelight extends SubsystemBase {

    // Constants
    private final static LimelightPipeline DefaultPipeline = LimelightPipeline.Shooting;

    // Devices
    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight-shooter");

    private final DoubleArraySubscriber camerapose_targetspaceSubscriber = limelight
            .getDoubleArrayTopic("camerapose_targetspace").subscribe(new double[] {});

    private final NetworkTableEntry getpipeEntry = limelight.getEntry("getpipe");
    private final NetworkTableEntry pipelineEntry = limelight.getEntry("pipeline");

    private final NetworkTableEntry tagIdEntry = limelight.getEntry("tid");
    private final NetworkTableEntry targetXEntry = limelight.getEntry("tx");
    private final NetworkTableEntry targetYEntry = limelight.getEntry("ty");
    private final NetworkTableEntry targetAreaEntry = limelight.getEntry("ta");

    // State
    private Timer activePipelineTimer = new Timer();
    private LimelightPipeline desiredPipeline = DefaultPipeline;
    private long activePipeline = 0;

    private static AprilLimelight instance = new AprilLimelight();

    public static AprilLimelight get() {
        return instance;
    }
    

    private AprilLimelight() {
        // super(robotContainer);

        activePipelineTimer.restart();

        try {
            HttpCamera limelightFeed = new HttpCamera("limelight-shooter", "http://10.34.89.103:5800/stream.mjg");//http://10.34.89.11:5800/stream.mjpg

            Shuffleboard.getTab("Main")
                    .add(limelightFeed)
                    .withWidget(BuiltInWidgets.kCameraStream);
        } catch (Exception e) {
            System.out.println("Limelight camera had trouble initializing");
        }
        Shuffleboard.getTab("Main").addDouble("Tag", () -> getTargetX());

        Shuffleboard.getTab("Testing")
                .addDouble("Limelight X", () -> getTargetX())
                .withSize(1, 1)
                .withPosition(5, 2);

        Shuffleboard.getTab("Testing")
                .addDouble("Limelight Y", () -> getTargetY())
                .withSize(1, 1)
                .withPosition(2, 2);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            setDesiredPipeline(DefaultPipeline);
        }

        long getpipe = getpipeEntry.getInteger(-1);
        if (getpipe != activePipeline) {
            activePipeline = getpipe;
            activePipelineTimer.restart();
        }

        if (!isActivePipeline(desiredPipeline)) {
            pipelineEntry.setNumber(desiredPipeline.getIndex());
        }
    }

    public Pose3d getCampose() {
        double[] campose = camerapose_targetspaceSubscriber.get();
        if (campose.length != 0) {
            Translation3d translationMeters = new Translation3d(campose[0], campose[1], campose[2]);
            Rotation3d rotation = new Rotation3d(campose[3], campose[4], campose[5]);
            return new Pose3d(translationMeters, rotation);
        } else {
            return null;
        }
    }

    public long getActivePipeline() {
        return activePipeline;
    }

    public boolean isActivePipeline(LimelightPipeline pipeline) {
        return activePipeline == pipeline.getIndex();
    }

    public void setDesiredPipeline(LimelightPipeline pipeline) {
        desiredPipeline = pipeline;
    }

    // Gets the ID of the Tag
    public long getTagId() {
        return tagIdEntry.getInteger(-1);
    }

    public double getTargetX() {
        return targetXEntry.getDouble(Double.NaN);
    }

    public double getTargetY() {
        return targetYEntry.getDouble(Double.NaN);
    }

    public double getTargetArea() {
        return targetAreaEntry.getDouble(Double.NaN);
    }

    public void printTargetData() {
        StringBuilder builder = new StringBuilder();
        builder.append("Limelight target data:\n");
        builder.append("\ttid: " + getTagId() + "\n");
        builder.append("\ttx: " + getTargetX() + "\n");
        builder.append("\tty: " + getTargetY() + "\n");
        builder.append("\tta: " + getTargetArea());
        System.out.println(builder.toString());
    }

}
