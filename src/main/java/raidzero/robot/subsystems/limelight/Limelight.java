package raidzero.robot.subsystems.limelight;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.lib.LimelightHelpers;
import raidzero.robot.subsystems.swerve.Swerve;
import raidzero.robot.subsystems.swerve.TunerConstants;

public class Limelight extends SubsystemBase {
    public class LimelightState {
        public enum PIPELINE {
            TAG,
            OBJECT,
        }

        private String limelightName;
        private LimelightHelpers.PoseEstimate currEstimate;
        private LimelightHelpers.PoseEstimate prevEstimate;
        private StructPublisher<Pose2d> ntPublisher;
        private boolean ignore;
        private PIPELINE pipeline = PIPELINE.TAG;

        /**
         * Constructs a LimelightState instance
         * 
         * @param limelightName The name of the limelight
         */
        public LimelightState(String limelightName) {
            this.limelightName = limelightName;
            this.ntPublisher = NetworkTableInstance.getDefault()
                    .getStructTopic(limelightName + "NT", Pose2d.struct)
                    .publish();

            this.ignore = false;
        }

        /**
         * Gets the current pose estimate
         * 
         * @return The current {@link LimelightHelpers.PoseEstimate}
         */
        public LimelightHelpers.PoseEstimate getEstimate() {
            return currEstimate;
        }

        /**
         * Gets the name of the limelight
         * 
         * @return The name of the limelight
         */
        public String getName() {
            return limelightName;
        }

        /**
         * Gets the current pipeline
         * 
         * @return The current {@link PIPELINE}
         */
        public PIPELINE getPipeline() {
            return pipeline;
        }

        /**
         * Sets the current pipeline
         * 
         * @param pipeline The {@link PIPELINE} to set
         */
        public void setPipeline(PIPELINE pipeline) {
            this.pipeline = pipeline;
            LimelightHelpers.setPipelineIndex(limelightName, pipeline.ordinal());
        }

        /**
         * Updates the limelight state
         * 
         * @param ignoreAll Whether to ignore all measurements
         */
        public void update(boolean ignoreAll) {
            update(ignoreAll, 0.5, 0.5, Units.degreesToRadians(5));
        }

        /**
         * Updates the limelight state
         * 
         * @param ignoreAll Whether to ignore all measurements
         * @param stdevX    The standard deviation in the X direction in meters
         * @param stdevY    The standard deviation in the Y direction in meters
         */
        public void update(boolean ignoreAll, double stdevX, double stdevY) {
            update(ignoreAll, stdevX, stdevY, Units.degreesToRadians(5));
        }

        /**
         * Updates the limelight state
         * 
         * @param ignoreAll Whether to ignore all measurements
         * @param stdevRot  The standard deviation for rotations in radians
         */
        public void update(boolean ignoreAll, double stdevRot) {
            update(ignoreAll, 0.5, 0.5, stdevRot);
        }

        /**
         * Updates the limelight state
         * 
         * @param ignoreAll Whether to ignore all measurements
         * @param stdevX    The standard deviation in the X direction in meters
         * @param stdevY    The standard deviation in the Y direction in meters
         * @param stdevRot  The standard deviation for rotations in radians
         */
        public void update(boolean ignoreAll, double stdevX, double stdevY, double stdevRot) {
            LimelightHelpers.SetRobotOrientation(
                    limelightName,
                    swerve.getState().Pose.getRotation().getDegrees(),
                    swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(),
                    0,
                    0,
                    0,
                    0);

            currEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

            if (currEstimate != null && currEstimate.pose != null) {
                ignore = checkIgnore();

                ntPublisher.set(currEstimate.pose);

                if (!ignore && !ignoreAll) {
                    swerve.addVisionMeasurement(
                            currEstimate.pose,
                            Utils.fpgaToCurrentTime(currEstimate.timestampSeconds),
                            VecBuilder.fill(stdevX, stdevY, stdevRot)
                                    .div(Math.max(LimelightHelpers.getTA(limelightName), -1)));
                }
            }

            prevEstimate = currEstimate;
        }

        private boolean checkIgnore() {
            return !poseInField(currEstimate.pose) ||
                    (Math.abs(LimelightHelpers.getBotPose3d_wpiBlue(limelightName).getZ()) > 0.4) ||
                    (LimelightHelpers.getTA(limelightName) < 0.1) ||
                    (prevEstimate != null
                            && (currEstimate.pose.getTranslation().getDistance(prevEstimate.pose.getTranslation()) /
                                    (currEstimate.timestampSeconds
                                            - prevEstimate.timestampSeconds)) > TunerConstants.kSpeedAt12Volts
                                                    .baseUnitMagnitude())
                    ||
                    (prevEstimate != null && (currEstimate.pose.getTranslation()
                            .getDistance(prevEstimate.pose
                                    .getTranslation()) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() * 0.02))
                    ||
                    (currEstimate.rawFiducials.length > 0 && currEstimate.rawFiducials[0].ambiguity > 0.5 &&
                            currEstimate.rawFiducials[0].distToCamera > 4.0)
                    ||
                    currEstimate.pose.equals(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        }

        /**
         * Checks if a pose is inside the field dimensions
         *
         * @param pose The {@link Pose2d} to check
         * @return True if the pose is inside the field dimensions, false otherwise
         */
        private boolean poseInField(Pose2d pose) {
            return pose.getTranslation().getX() > 0 &&
                    pose.getTranslation().getX() < 17.55 &&
                    pose.getTranslation().getY() > 0 &&
                    pose.getTranslation().getY() < 8.05;
        }
    }

    private LimelightState flState = new LimelightState("limelight-fl");
    private LimelightState frState = new LimelightState("limelight-fr");
    private LimelightState blState = new LimelightState("limelight-bl");
    private LimelightState brState = new LimelightState("limelight-br");

    private boolean ignoreAllLimes = false;

    private Notifier notifier;

    private Swerve swerve = Swerve.system();
    private static Limelight instance = null;

    /**
     * Constructs a {@link Limelight} subsytem instance
     */
    private Limelight() {
        this.startThread();
    }

    /**
     * Starts the Limelight odometry thread
     */
    private void startThread() {
        notifier = new Notifier(this::loop);
        notifier.startPeriodic(0.02);
    }

    /**
     * The main loop of the Limelight odometry thread
     */
    private void loop() {
        if (swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble() > 720) {
            ignoreAllLimes = true;
        } else {
            ignoreAllLimes = false;
        }

        flState.update(ignoreAllLimes);
        frState.update(ignoreAllLimes);
        blState.update(ignoreAllLimes, 0.75, 0.75);
        brState.update(ignoreAllLimes);
    }

    /**
     * /**
     * Gets the {@link Limelight} subsystem instance
     *
     * @return The {@link Limelight} subsystem instance
     */
    public static Limelight system() {
        if (instance == null) {
            instance = new Limelight();
        }

        return instance;
    }
}
