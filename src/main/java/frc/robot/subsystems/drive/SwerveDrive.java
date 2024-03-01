package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.Constants.SubSystems;
import frc.robot.subsystems.vision.VisionCamera.TimestampedVisionMeasurement;
import frc.robot.utils.FieldVisualizer;
import frc.robot.utils.Print;
import frc.robot.utils.TimerUtil;
import frc.robot.utils.TunableNumber;

public class SwerveDrive extends SubsystemBase {
    public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(15.1);
    public static final double TELEOP_DRIVE_MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(15.1); // ?
    public static final double TELEOP_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI; // ?
    
    public static final TunableNumber max_linear_acceleration_per_second = new TunableNumber("Drive/MaxDriveLinearAcceleration", Units.feetToMeters(4));
    public static final TunableNumber max_angular_acceleration_per_second = new TunableNumber("Drive/MaxDriveAngularAcceleration", Math.PI / 2);
    
    private static final edu.wpi.first.wpilibj.SerialPort.Port NAVX_PORT = edu.wpi.first.wpilibj.SerialPort.Port.kUSB;
    private static final double CENTER_TO_MODULE = Units.inchesToMeters(10.75);
    private static final Matrix<N3, N1> POSITION_STD_DEV = VecBuilder.fill(0.1, 0.1, 0.1);
    private static final Matrix<N3, N1> VISION_STD_DEV   = VecBuilder.fill(1, 1, 3);
    private static final int MAX_NAVX_CALIBRATION_TIME_MS = 20 * 1000;
    private static final String[] module_names = { "Front Right", "Front Left", "Back Right", "Back Left" };
    private static final double[] abs_encoder_offsets = { 4.071693, 2.830042 + Math.PI, 5.274043, 1.992770 + Math.PI};
    
    private static final Translation2d front_right_position = new Translation2d(CENTER_TO_MODULE,  -CENTER_TO_MODULE); // (+, -)
    private static final Translation2d front_left_position  = new Translation2d(CENTER_TO_MODULE,   CENTER_TO_MODULE); // (+, +)
    private static final Translation2d back_right_position  = new Translation2d(-CENTER_TO_MODULE, -CENTER_TO_MODULE); // (-, -)
    private static final Translation2d back_left_position   = new Translation2d(-CENTER_TO_MODULE,  CENTER_TO_MODULE); // (-, +) 
    private static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        front_right_position,
        front_left_position,
        back_right_position,
        back_left_position );
    private static final SwerveDriveKinematicsConstraint kinematics_constrait = new SwerveDriveKinematicsConstraint(kinematics, PHYSICAL_MAX_SPEED_METERS_PER_SECOND);

    private final NavX navx = Robot.isReal() ? new NavXReal(NAVX_PORT) : new NavXSim(NAVX_PORT);
    private final SwerveModule[] modules;
    private final SwerveDriveOdometry odometer;
    private final SwerveDrivePoseEstimator pose_estimator;

    private Pose2d robot_pose = new Pose2d(1.567501, 5.380708, Rotation2d.fromDegrees(180));

    public SwerveDrive(){
        modules = new SwerveModule[4];
        for(int i = 0; i < 4; i++)
            if(Robot.isReal()) modules[i] = new SwerveModuleReal(module_names[i], i*2 + 1, i*2 + 2, i, abs_encoder_offsets[i]);
            else modules[i] = new SwerveModuleSim(module_names[i], i);
        
        odometer = new SwerveDriveOdometry(kinematics, navx.getRotation2d(), getModulePositions(), robot_pose);
        pose_estimator = new SwerveDrivePoseEstimator(kinematics, navx.getRotation2d(),
            getModulePositions(),  
            robot_pose, 
            POSITION_STD_DEV,
            VISION_STD_DEV);

        new Thread(() -> {
            TimerUtil calibration_timer = new TimerUtil();
            while(navx.isCalibrating()) 
                if(calibration_timer.hasTimeElapsed(MAX_NAVX_CALIBRATION_TIME_MS, false))
                    break;
            navx.reset();
            navx.resetDisplacement();
        }).start();
        Logger.recordOutput("PoseEstimation/VisionMeasurement", new Pose2d());
    }

    @Override public void periodic(){
        for (var module : modules) module.periodic();
        navx.periodic();
        var vision_measurements = SubSystems.vision.getAllVisionMeasurements();
        for (TimestampedVisionMeasurement vision_measurement : vision_measurements){
            pose_estimator.addVisionMeasurement(vision_measurement.pose, vision_measurement.timestamp, vision_measurement.std_devs);
            Logger.recordOutput("PoseEstimation/VisionMeasurement", vision_measurement.pose);
        }
        odometer.update(navx.getRotation2d(), getWheelPositions());
        pose_estimator.update(
            navx.getRotation2d(),
            getWheelPositions());

        robot_pose = pose_estimator.getEstimatedPosition();

        FieldVisualizer.getField().setRobotPose(odometer.getPoseMeters());
        FieldVisualizer.getField().getObject("SwerveModules").setPoses(
            robot_pose.transformBy(new Transform2d(front_right_position, modules[0].getAngle())),
            robot_pose.transformBy(new Transform2d(front_left_position, modules[1].getAngle())),
            robot_pose.transformBy(new Transform2d(back_right_position, modules[2].getAngle())),
            robot_pose.transformBy(new Transform2d(back_left_position, modules[3].getAngle()))
            );
        
        // FieldVisualizer.getField().getObject("Cameras").setPoses(SubSystems.vision.getAllRobotToCameraPoses(robot_pose));

        Logger.recordOutput("Drive/Modules/States/Actual", getModuleStates());
        Logger.recordOutput("Drive/Modules/Positions", getModulePositions());
        Logger.recordOutput("PoseEstimation/Odometer", odometer.getPoseMeters());
        Logger.recordOutput("PoseEstimation/NavX-Displacement", navx.getPose3d());
        Logger.recordOutput("PoseEstimation/NavX-Odometer", navx.getEstimatedPose());
        Logger.recordOutput("PoseEstimation/FullPoseEstimator", robot_pose);
        // Logger.recordOutput("DriverCamPose", 
            // new Pose3d(robot_pose).plus(SubSystems.vision.getCamera("DriverCam").getTransform()));
    }

    private double getClosestToTarget(double target, double[] values) {
        double closestValue = values[0];
        double leastDistance = Math.abs(values[0] - target);
        for (int i = 0; i < values.length; i++) {
            double currentDistance = Math.abs(values[i] - target);
            if (currentDistance < leastDistance) {
                closestValue = values[i];
                leastDistance = currentDistance;
            }
        }
        return closestValue;
    }
    public void postEnableSetup(){
        navx.setStartPose(robot_pose);
        setHeadingDefault();
    }
    public void setHeadingDefault(){
        Pose2d held_pose = robot_pose;
        Print.error("Fix this gng?");
        if(FieldConstants.isBlueAlliance()){
            Print.log("Heading Blue Alliance");
            final double robot_rotation = robot_pose.getRotation().getDegrees();
            final double[] rotations = new double[]{-120.0, 180.0, -180.0, 120.0};
            final double target_rotation = getClosestToTarget(robot_rotation, rotations);
            switch ((int)target_rotation) {
                case -120: setHeading(Rotation2d.fromDegrees(120)); break;
                case -180:
                case 180: setHeading(Rotation2d.fromDegrees(180)); break;
                case 120: setHeading(Rotation2d.fromDegrees(-120)); break;
                default: Print.error("Blue Alliance Unknown Rotation"); break;
            }
        } else { // Red Alliances
            Print.log("Heading Red Alliance Selected");
            final double robot_rotation = robot_pose.getRotation().getDegrees();
            final double[] rotations = new double[]{-60.0, 0.0, 60.0};
            final double target_rotation = getClosestToTarget(robot_rotation, rotations);
            switch ((int)target_rotation) {
                case -60: setHeading(Rotation2d.fromDegrees(180-60)); break;
                case 0: setHeading(Rotation2d.fromDegrees(180)); break;
                case 60: setHeading(Rotation2d.fromDegrees(180+60)); break;
                default: Print.error("Red Alliance Unknown Rotation"); break;
            }
        }
        odometer.resetPosition(navx.getRotation2d(), getWheelPositions(), held_pose);
        pose_estimator.resetPosition(
            navx.getRotation2d(),
            getWheelPositions(),
            held_pose);
    }
    public void setHeading(Rotation2d rotation){ navx.setAngleAdjustment(rotation.getDegrees()); }
    public Rotation2d getHeading(){ return navx.getRotation2d(); }

    private SwerveDriveWheelPositions getWheelPositions(){
        return new SwerveDriveWheelPositions(getModulePositions());
    }
    private SwerveModuleState[] getModuleStates(){
        return new SwerveModuleState[] {
                modules[0].getState(),
                modules[1].getState(),
                modules[2].getState(),
                modules[3].getState()};
    }
    public SwerveModulePosition[] getModulePositions(){
        return new SwerveModulePosition[] {
                modules[0].getPosition(),
                modules[1].getPosition(),
                modules[2].getPosition(),
                modules[3].getPosition()};
    }
    public SwerveModule[] getModules(){ synchronized(modules){ return modules; } }
    public Pose2d getPose(){ return robot_pose; }
    public SwerveDriveKinematics getKinematics(){ return kinematics; }
    public SwerveDriveKinematicsConstraint getKinematicsConstraint(){ return kinematics_constrait; }

    public void stopModules(){ for(int i = 0; i < 4; i++) modules[i].stop(); }
    public void xStopModules(){ for(int i = 0; i < 4; i++) modules[i].xStop(); }
    public void setModuleStates(SwerveModuleState[] desired_states){
        SwerveDriveKinematics.desaturateWheelSpeeds(desired_states, PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
        Logger.recordOutput("Drive/Modules/States/Desired", desired_states);
        for(int i = 0; i < 4; i++) modules[i].runSetpoint(desired_states[i]);
    }
}