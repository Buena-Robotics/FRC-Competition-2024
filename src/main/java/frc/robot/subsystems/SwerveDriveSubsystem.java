package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.FieldPoses;
import frc.robot.utils.TimerUtil;
import frc.robot.vendor.NavX;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class SwerveDriveSubsystem extends SubsystemBase {
    private static final int // DRIVE_MOTOR_IDS : ODDS
        FRONT_RIGHT_DRIVE_MOTOR_ID = 1,
        FRONT_LEFT_DRIVE_MOTOR_ID  = 3,
        BACK_RIGHT_DRIVE_MOTOR_ID  = 5,
        BACK_LEFT_DRIVE_MOTOR_ID   = 7;
    private static final int // TURN_MOTOR_IDS : EVENS
        FRONT_RIGHT_TURN_MOTOR_ID = 2,
        FRONT_LEFT_TURN_MOTOR_ID  = 4,
        BACK_RIGHT_TURN_MOTOR_ID  = 6,
        BACK_LEFT_TURN_MOTOR_ID   = 8;
    private static final boolean // DRIVE_MOTER_REVERSED
        FRONT_RIGHT_DRIVE_MOTER_REVERSED = false,
        FRONT_LEFT_DRIVE_MOTER_REVERSED  = false,
        BACK_RIGHT_DRIVE_MOTER_REVERSED  = false,
        BACK_LEFT_DRIVE_MOTER_REVERSED   = false;
    private static final boolean // TURN_MOTOR_REVERSED
        FRONT_RIGHT_TURN_MOTOR_REVERSED = false,
        FRONT_LEFT_TURN_MOTOR_REVERSED  = false,
        BACK_RIGHT_TURN_MOTOR_REVERSED  = false,
        BACK_LEFT_TURN_MOTOR_REVERSED   = false;
    private static final int // ABSOLUTE_ENCODER_IDS : 0-3
        FRONT_RIGHT_ABSOLUTE_ENCODER_ID = 0,
        FRONT_LEFT_ABSOLUTE_ENCODER_ID  = 1,
        BACK_RIGHT_ABSOLUTE_ENCODER_ID  = 2,
        BACK_LEFT_ABSOLUTE_ENCODER_ID   = 3;
    private static final double // ABSOLUTE_ENCODER_OFFSET_RADIANS
        FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS = 4.071693,
        FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS  = 5.996429,
        BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS  = 5.274043,
        BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS   = 1.978168;
    private static final boolean // ABSOLUTE_ENCODER_REVERSED
        FRONT_RIGHT_ABSOLUTE_ENCODER_REVERSED = false,
        FRONT_LEFT_ABSOLUTE_ENCODER_REVERSED  = false,
        BACK_RIGHT_ABSOLUTE_ENCODER_REVERSED  = false,
        BACK_LEFT_ABSOLUTE_ENCODER_REVERSED   = false;
    private static final int MAX_NAVX_CALIBRATION_TIME_MS = 20 * 1000;

    private final SwerveModule front_right = new SwerveModule (FRONT_RIGHT_DRIVE_MOTOR_ID, FRONT_RIGHT_TURN_MOTOR_ID, FRONT_RIGHT_DRIVE_MOTER_REVERSED, FRONT_RIGHT_TURN_MOTOR_REVERSED, 
                                                                FRONT_RIGHT_ABSOLUTE_ENCODER_ID, FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS, FRONT_RIGHT_ABSOLUTE_ENCODER_REVERSED );
    private final SwerveModule front_left  = new SwerveModule (FRONT_LEFT_DRIVE_MOTOR_ID, FRONT_LEFT_TURN_MOTOR_ID, FRONT_LEFT_DRIVE_MOTER_REVERSED, FRONT_LEFT_TURN_MOTOR_REVERSED, 
                                                                FRONT_LEFT_ABSOLUTE_ENCODER_ID, FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS, FRONT_LEFT_ABSOLUTE_ENCODER_REVERSED );
    private final SwerveModule back_right  = new SwerveModule (BACK_RIGHT_DRIVE_MOTOR_ID, BACK_RIGHT_TURN_MOTOR_ID, BACK_RIGHT_DRIVE_MOTER_REVERSED, BACK_RIGHT_TURN_MOTOR_REVERSED,
                                                                BACK_RIGHT_ABSOLUTE_ENCODER_ID, BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS, BACK_RIGHT_ABSOLUTE_ENCODER_REVERSED);
    private final SwerveModule back_left   = new SwerveModule (BACK_LEFT_DRIVE_MOTOR_ID, BACK_LEFT_TURN_MOTOR_ID, BACK_LEFT_DRIVE_MOTER_REVERSED, BACK_LEFT_TURN_MOTOR_REVERSED,
                                                                BACK_LEFT_ABSOLUTE_ENCODER_ID, BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS, BACK_LEFT_ABSOLUTE_ENCODER_REVERSED);
    //-------------------------
    private final NavX gyro = new NavX(edu.wpi.first.wpilibj.SerialPort.Port.kUSB2);
    // private final ColorSensorV3 color_sensor = new ColorSensorV3(I2C.Port.kOnboard);

    //Distance between right and left wheels
    private static final double TRACK_WIDTH_CENTER = Units.inchesToMeters(10.75);
    //Distance between front and back wheels
    private static final double WHEEL_BASE_CENTER = Units.inchesToMeters(10.75);

    private static final Translation2d front_left_position =  new Translation2d(WHEEL_BASE_CENTER, TRACK_WIDTH_CENTER / 2);
    private static final Translation2d front_right_position = new Translation2d(WHEEL_BASE_CENTER / 2, -TRACK_WIDTH_CENTER / 2);
    private static final Translation2d back_left_position =   new Translation2d(-WHEEL_BASE_CENTER / 2, TRACK_WIDTH_CENTER / 2);
    private static final Translation2d back_right_position =  new Translation2d(-WHEEL_BASE_CENTER / 2, -TRACK_WIDTH_CENTER / 2);

    //Might have to reorder these :3
    public static final SwerveDriveKinematics swerve_kinematics = new SwerveDriveKinematics(
        front_left_position,
        front_right_position,
        back_left_position,
        back_right_position
    );
    SwerveDriveOdometry swerve_odometry = new SwerveDriveOdometry(
      swerve_kinematics, gyro.getRotation2d(),
      new SwerveModulePosition[] {
        front_right.getPosition(),
        front_left.getPosition(),
        back_right.getPosition(),
        back_left.getPosition()
    }, new Pose2d(0, 0, new Rotation2d()));

    private final Field2d glass_field = new Field2d();

    private final Translation3d robot_to_cam_translation = new Translation3d(Units.inchesToMeters(FieldPoses.ROBOT_LENGTH / 2), 0, 0);
    private final Rotation3d robot_to_cam_rotation = new Rotation3d(0,Units.degreesToRadians(15),0);
    private final Transform3d robot_to_cam = new Transform3d(robot_to_cam_translation, robot_to_cam_rotation);
    private final PhotonCamera camera;
    private final PhotonPoseEstimator pose_estimator;
    private AprilTagFieldLayout field_layout;
    private boolean april_tag_lock = false;
    private boolean pose_known = false;
    private Pose2d robot_pose = new Pose2d();
    private Pose2d target_pose = new Pose2d();

    public SwerveDriveSubsystem(){
        SmartDashboard.putData("Field", glass_field);
        SmartDashboard.putData("Swerve/Nav-X", gyro);
        SmartDashboard.putData("Swerve/Front Right", front_right);
        SmartDashboard.putData("Swerve/Front Left" , front_left);
        SmartDashboard.putData("Swerve/Back Right" , back_right);
        SmartDashboard.putData("Swerve/Back Left"  , back_left);
        SmartDashboard.putBoolean("April Tag Lock", april_tag_lock);

        try { field_layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile); } catch (Exception e) { System.err.println(e); }
        initializeGlassFieldTags();

        camera = new PhotonCamera("Microsoft_LifeCam_HD-3000"); // new PhotonCamera("Microsoft_LifeCam_HD-3000");
        pose_estimator = new PhotonPoseEstimator(field_layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robot_to_cam);
        pose_estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        new Thread(() -> {
            try {
                Thread.sleep(1000);
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

    private void initializeGlassFieldTags(){
        Pose2d[] tag_poses = new Pose2d[16];
        for(int i = 1; i <= 16; i++){
            tag_poses[i-1] = field_layout.getTagPose(i).get().toPose2d();
        }
        glass_field.getObject("AprilTags").setPoses(tag_poses);
    }

    public Field2d getField2d(){ return glass_field; }
    public boolean getPoseKnown(){ return pose_known; }
    public Pose2d getRobotPose(){ return robot_pose; }
    public void setRobotPose(Pose2d pose){ robot_pose = pose; }
    public Pose2d getTargetPose(){ return Robot.isReal() ? target_pose : glass_field.getObject("AprilTags").getPoses().get(5); }
    public void toggleAprilTags(){ april_tag_lock = !april_tag_lock; }

    public void zeroHeading(){ gyro.reset(); }
    public double getHeading(){ return Math.IEEEremainder(gyro.getAngle(), 360); }

    public Rotation2d getRotation2d(){ return Rotation2d.fromDegrees(getHeading()); }

    @Override public void periodic() { 
        if(camera.isConnected()){
            PhotonPipelineResult result = camera.getLatestResult();
            if(result.hasTargets() && result.getBestTarget().getPoseAmbiguity() < 0.2 && !april_tag_lock){
                target_pose = field_layout.getTagPose(result.getBestTarget().getFiducialId()).get().toPose2d();            
                Optional<EstimatedRobotPose> opt_estimated_pose = pose_estimator.update(result);
                if(!opt_estimated_pose.isEmpty()){
                    EstimatedRobotPose estimated_pose = opt_estimated_pose.get(); 
                    Pose2d estimated_pose_2d = estimated_pose.estimatedPose.toPose2d();
                    robot_pose = estimated_pose_2d;
                    glass_field.setRobotPose(robot_pose);
                    
                    gyro.setAngleAdjustment(estimated_pose_2d.getRotation().getDegrees() - gyro.getAngle());
    
                    front_right.resetPosition();
                    front_left.resetPosition();
                    back_right.resetPosition();
                    back_left.resetPosition();
    
                    swerve_odometry.resetPosition(gyro.getRotation2d(), new SwerveModulePosition[] {
                        front_right.getPosition(),
                        front_left.getPosition(),
                        back_right.getPosition(),
                        back_left.getPosition()
                    }, estimated_pose_2d);
                    pose_known = true;
                }
            } else if(pose_known){
                robot_pose = swerve_odometry.update(gyro.getRotation2d(), new SwerveModulePosition[] {
                    front_left.getPosition(),
                    front_right.getPosition(),
                    back_left.getPosition(),
                    back_right.getPosition()
                });
                glass_field.setRobotPose(robot_pose);
            }
        }
        if(pose_known) glass_field.getObject("RobotToTarget").setPoses(robot_pose, target_pose);
    }
    @Override public void simulationPeriodic(){}

    public void stopModules(){
        front_left.stop();
        front_right.stop();
        back_left.stop();
        back_right.stop();
    }

    public void setModuleStates(SwerveModuleState[] desired_states){
        assert desired_states.length == 4 : "desired_states has invalid length";
        SwerveDriveKinematics.desaturateWheelSpeeds(desired_states, Constants.Drive.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
        front_left.setDesiredState (desired_states[0]);
        front_right.setDesiredState(desired_states[1]);
        back_left.setDesiredState  (desired_states[2]);
        back_right.setDesiredState (desired_states[3]);
    }
}
