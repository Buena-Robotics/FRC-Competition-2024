package frc.robot.subsystems;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.robot.SwerveModule;
import frc.robot.subsystems.simulation.SwerveModuleSim;
import frc.robot.Constants.FieldPoses;
import frc.robot.devices.NavX;
import frc.robot.utils.TimerUtil;

public abstract class SwerveDriveIO extends SubsystemBase {
    protected static final int // DRIVE_MOTOR_IDS : ODDS
        FRONT_RIGHT_DRIVE_MOTOR_ID = 1,
        FRONT_LEFT_DRIVE_MOTOR_ID  = 3,
        BACK_RIGHT_DRIVE_MOTOR_ID  = 5,
        BACK_LEFT_DRIVE_MOTOR_ID   = 7;
    protected static final int // TURN_MOTOR_IDS : EVENS
        FRONT_RIGHT_TURN_MOTOR_ID = 2,
        FRONT_LEFT_TURN_MOTOR_ID  = 4,
        BACK_RIGHT_TURN_MOTOR_ID  = 6,
        BACK_LEFT_TURN_MOTOR_ID   = 8;
    protected static final int // ABSOLUTE_ENCODER_IDS : 0-3
        FRONT_RIGHT_ABSOLUTE_ENCODER_ID = 0,
        FRONT_LEFT_ABSOLUTE_ENCODER_ID  = 1,
        BACK_RIGHT_ABSOLUTE_ENCODER_ID  = 2,
        BACK_LEFT_ABSOLUTE_ENCODER_ID   = 3;
    protected static final double // ABSOLUTE_ENCODER_OFFSET_RADIANS
        FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS = 0.975905,
        FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS  = 2.845395,
        BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS  = 2.04,
        BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS   = 5.131805;
    protected static final int MAX_NAVX_CALIBRATION_TIME_MS = 20 * 1000;
    protected static final I2C.Port NAVX_PORT = I2C.Port.kOnboard;

    protected static final double TRACK_WIDTH = Units.inchesToMeters(20); //Distance between right and left wheels
    protected static final double WHEEL_BASE = Units.inchesToMeters(20); //Distance between front and back wheels
    protected static final Translation2d FRONT_RIGHT_POSITION = new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2);
    protected static final Translation2d FRONT_LEFT_POSITION = new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2);
    protected static final Translation2d BACK_RIGHT_POSITION = new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2);
    protected static final Translation2d BACK_LEFT_POSITION = new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2);

    protected static final Translation3d CAMERA_TO_ROBOT_TRANSLATION = new Translation3d(Units.inchesToMeters(FieldPoses.ROBOT_LENGTH), 0, Units.inchesToMeters(12));
    protected static final Rotation3d CAMERA_TO_ROBOT_ROTATION = new Rotation3d(0,Units.degreesToRadians(-10),0);
    protected static final Transform3d CAMERA_TO_ROBOT = new Transform3d(CAMERA_TO_ROBOT_TRANSLATION, CAMERA_TO_ROBOT_ROTATION);

    protected final NavX gyro = new NavX(NAVX_PORT);
    protected final SwerveModuleIO front_right;
    protected final SwerveModuleIO front_left;
    protected final SwerveModuleIO back_right;
    protected final SwerveModuleIO back_left;

    protected final SwerveDriveKinematics swerve_kinematics = new SwerveDriveKinematics(
        FRONT_RIGHT_POSITION,
        FRONT_LEFT_POSITION,
        BACK_RIGHT_POSITION,
        BACK_LEFT_POSITION
    );

    protected final SwerveDrivePoseEstimator swerve_pose_estimator;

    protected final Field2d glass_field = new Field2d();

    protected final PhotonCamera photon_camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    protected final PhotonPoseEstimator photon_pose_estimator;
    protected AprilTagFieldLayout field_layout;
    protected double previous_vision_result_timestamp = 0.0;
    protected Pose2d trajectory_target_pose = new Pose2d();
    protected ArrayList<Pose2d> tracked_target_poses = new ArrayList<Pose2d>();

    protected SwerveDriveIO(){
        if(Robot.isReal()){
            front_right = new SwerveModule (FRONT_RIGHT_DRIVE_MOTOR_ID, FRONT_RIGHT_TURN_MOTOR_ID, FRONT_RIGHT_ABSOLUTE_ENCODER_ID, FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS);
            front_left  = new SwerveModule (FRONT_LEFT_DRIVE_MOTOR_ID, FRONT_LEFT_TURN_MOTOR_ID, FRONT_LEFT_ABSOLUTE_ENCODER_ID, FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS);
            back_right  = new SwerveModule (BACK_RIGHT_DRIVE_MOTOR_ID, BACK_RIGHT_TURN_MOTOR_ID, BACK_RIGHT_ABSOLUTE_ENCODER_ID, BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS);
            back_left   = new SwerveModule (BACK_LEFT_DRIVE_MOTOR_ID, BACK_LEFT_TURN_MOTOR_ID, BACK_LEFT_ABSOLUTE_ENCODER_ID, BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS);
        } else{
            front_right = new SwerveModuleSim (FRONT_RIGHT_DRIVE_MOTOR_ID, FRONT_RIGHT_TURN_MOTOR_ID, FRONT_RIGHT_ABSOLUTE_ENCODER_ID, FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS);
            front_left  = new SwerveModuleSim (FRONT_LEFT_DRIVE_MOTOR_ID, FRONT_LEFT_TURN_MOTOR_ID, FRONT_LEFT_ABSOLUTE_ENCODER_ID, FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS);
            back_right  = new SwerveModuleSim (BACK_RIGHT_DRIVE_MOTOR_ID, BACK_RIGHT_TURN_MOTOR_ID, BACK_RIGHT_ABSOLUTE_ENCODER_ID, BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS);
            back_left   = new SwerveModuleSim (BACK_LEFT_DRIVE_MOTOR_ID, BACK_LEFT_TURN_MOTOR_ID, BACK_LEFT_ABSOLUTE_ENCODER_ID, BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS);
        }
        swerve_pose_estimator = new SwerveDrivePoseEstimator(swerve_kinematics, gyro.getRotation2d(),
            new SwerveModulePosition[] {
                front_right.getPosition(),
                front_left.getPosition(),
                back_right.getPosition(),
                back_left.getPosition()
        },  new Pose2d(2.274279, 1.726919, Rotation2d.fromDegrees(45)), 
            VecBuilder.fill(0.1, 0.1, 0.1), 
            VecBuilder.fill(1, 1, 1)
        );

        try { field_layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile); } catch (Exception e) { System.err.println(e); }
        glassInitializeFieldTags();
        initializeNTSendables();

        photon_pose_estimator = new PhotonPoseEstimator(field_layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photon_camera, CAMERA_TO_ROBOT);
        photon_pose_estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        new Thread(() -> {
            try {
                TimerUtil calibration_timer = new TimerUtil();
                while(gyro.isCalibrating()){ 
                    if(calibration_timer.hasTimeElapsed(MAX_NAVX_CALIBRATION_TIME_MS, false)){
                        System.err.println("Nav-X Unable to Calibrate in alloted time of 20 seconds");
                        break;
                    }
                }
                zeroHeading();
            } catch (Exception e) { }
        }).start();
    }

    public abstract Pose2d getTargetPose();
    public abstract void setVisionMeasurementMultiTarget(PhotonPipelineResult pipeline_result, double result_timestamp);
    public abstract void setVisionMeasurementSingleTarget(PhotonPipelineResult pipeline_result, double result_timestamp);
    public abstract void setVisionMeasurement();
    public abstract void updatePoseEstimator();
    
    public Field2d getField2d()  { return glass_field; }
    public Pose2d getRobotPose() { return swerve_pose_estimator.getEstimatedPosition(); }

    public SwerveDriveKinematics getKinematics(){ return swerve_kinematics; }

    public void zeroHeading(){ gyro.reset(); }
    public double getHeading(){ return Math.IEEEremainder(gyro.getAngle(), 360); }

    public Rotation2d getRotation2d(){ return Rotation2d.fromDegrees(getHeading()); }
    public NavX getGyro(){ return gyro; }

    protected void initializeNTSendables(){
        SmartDashboard.putData("Field", glass_field);
        SmartDashboard.putData("Swerve Drive/Front Left",front_left);
        SmartDashboard.putData("Swerve Drive/Front Right",front_right);
        SmartDashboard.putData("Swerve Drive/Back Left",back_left);
        SmartDashboard.putData("Swerve Drive/Back Right",back_right);
        SmartDashboard.putData("Swerve Drive/NavX", gyro);
    }

    protected void glassInitializeFieldTags(){
        Pose2d[] tag_poses = new Pose2d[16];
        for(int i = 1; i <= 16; i++)
            tag_poses[i-1] = field_layout.getTagPose(i).get().toPose2d();
        glass_field.getObject("AprilTags").setPoses(tag_poses);
    }

    public void stopModules(){
        front_left.stop(); 
        front_right.stop();
        back_left.stop(); 
        back_right.stop();
    }

    public void setModuleStates(SwerveModuleState[] desired_states){
        SwerveDriveKinematics.desaturateWheelSpeeds(desired_states, Constants.Drive.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
        front_left.setDesiredState (desired_states[0]);
        front_right.setDesiredState(desired_states[1]);
        back_left.setDesiredState  (desired_states[2]);
        back_right.setDesiredState (desired_states[3]);
    }

    @Override public void initSendable(SendableBuilder builder){}
}
