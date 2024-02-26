package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Robot;
import frc.robot.Constants.SubSystems;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.FieldVisualizer;
import frc.robot.utils.TimerUtil;
import frc.robot.utils.TunableNumber;

public class SwerveDrive extends SubsystemBase {
    public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(15.1);
    public static final double TELEOP_DRIVE_MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(15.1); // ?
    public static final double TELEOP_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI; // ?
    
    public static final TunableNumber max_linear_acceleration_per_second = new TunableNumber("Drive/MaxDriveLinearAcceleration", Units.feetToMeters(4));
    public static final TunableNumber max_angular_acceleration_per_second = new TunableNumber("Drive/MaxDriveAngularAcceleration", Math.PI / 2);
    
    private static final double CENTER_TO_MODULE = Units.inchesToMeters(10.75);
    private static final Matrix<N3, N1> POSITION_STD_DEV = VecBuilder.fill(0.1, 0.1, 0.1);
    private static final Matrix<N3, N1> VISION_STD_DEV   = VecBuilder.fill(1, 1, 3);
    private static final Matrix<N3, N1> VISION_FIRST_STD_DEV   = VecBuilder.fill(0.3, 0.3, 0.1);
    private static final int MAX_NAVX_CALIBRATION_TIME_MS = 20 * 1000;
    private static final String[] module_names = { "Front Right", "Front Left", "Back Right", "Back Left" };
    private static final double[] abs_encoder_offsets = { 4.071693, 2.830042 + Math.PI, 5.274043, 1.992770 + Math.PI };
    
    private static final Translation2d front_right_position = new Translation2d(CENTER_TO_MODULE,  -CENTER_TO_MODULE); // (+, -)
    private static final Translation2d front_left_position  = new Translation2d(CENTER_TO_MODULE,   CENTER_TO_MODULE); // (+, +)
    private static final Translation2d back_right_position  = new Translation2d(-CENTER_TO_MODULE, -CENTER_TO_MODULE); // (-, -)
    private static final Translation2d back_left_position   = new Translation2d(-CENTER_TO_MODULE,  CENTER_TO_MODULE); // (-, +)
    private static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        front_right_position,
        front_left_position,
        back_right_position,
        back_left_position
    );

    public final NavX navx = new NavX(edu.wpi.first.wpilibj.SerialPort.Port.kUSB);
    private final SwerveModule[] modules;
    private final SwerveDrivePoseEstimator pose_estimator;

    private Pose2d robot_pose = new Pose2d(1.567501, 5.380708, Rotation2d.fromDegrees(180));
    private boolean found_pose = false;

    public SwerveDrive(){
        modules = new SwerveModule[4];
        for(int i = 0; i < 4; i++)
            if(Robot.isReal()) modules[i] = new SwerveModuleReal(module_names[i], i*2 + 1, i*2 + 2, i, abs_encoder_offsets[i]);
            else modules[i] = new SwerveModuleSim(module_names[i], i);
        
        pose_estimator = new SwerveDrivePoseEstimator(kinematics, navx.getRotation2d(),
            new SwerveModulePosition[] {
                modules[0].getPosition(),
                modules[1].getPosition(),
                modules[2].getPosition(),
                modules[3].getPosition() },  
            robot_pose, 
            POSITION_STD_DEV, 
            VISION_STD_DEV );

        new Thread(() -> {
            TimerUtil calibration_timer = new TimerUtil();
            while(navx.isCalibrating()) 
                if(calibration_timer.hasTimeElapsed(MAX_NAVX_CALIBRATION_TIME_MS, false))
                    break;
            navx.reset();
            navx.resetDisplacement();
        }).start();
        // for(int i = 0; i < 4; i++)
            // SmartDashboard.putData(module_names[i], modules[i]);
        // FieldVisualizer.getField().getObject("Test").setPose(new Pose2d());
    }

    @Override public void periodic(){
        for (var module : modules) module.periodic();
        SubSystems.vision._periodic();
        var vision_measurements = SubSystems.vision.getVisionMeasurements();
        for (var vision_measurement : vision_measurements){
            if(!found_pose) {
                found_pose = true;
                // navx.setAngleAdjustment(vision_measurement.pose.getRotation().getDegrees() - navx.getAngle());
                pose_estimator.addVisionMeasurement(vision_measurement.pose, vision_measurement.timestamp, VISION_FIRST_STD_DEV);
            }
            else pose_estimator.addVisionMeasurement(vision_measurement.pose, vision_measurement.timestamp, VISION_STD_DEV);
            Logger.recordOutput("PoseEstimation/VisionMeasurement", vision_measurement.pose);
        }
        SwerveModuleState[] swerve_module_states = new SwerveModuleState[] {
                modules[0].getState(),
                modules[1].getState(),
                modules[2].getState(),
                modules[3].getState()};
        Logger.recordOutput("SwerveModules/SwerveModuleStates", swerve_module_states);
        robot_pose = pose_estimator.update(
            navx.getRotation2d(),
            new SwerveModulePosition[] {
                modules[0].getPosition(),
                modules[1].getPosition(),
                modules[2].getPosition(),
                modules[3].getPosition()} );
        Logger.recordOutput("PoseEstimation/PoseEstimation", robot_pose);
        Logger.recordOutput("CameraPoses/Test1", new Pose3d(robot_pose).plus(SubSystems.vision.camera_pose.inverse()));
        Logger.recordOutput("CameraPoses/Test2", new Pose3d(robot_pose).plus(SubSystems.vision.camera_pose));
        Logger.recordOutput("CameraPoses/Test3", new Pose3d(robot_pose).transformBy(SubSystems.vision.camera_pose));

        FieldVisualizer.getField().setRobotPose(robot_pose);
        FieldVisualizer.getField().getObject("SwerveModules").setPoses(
            robot_pose.transformBy(new Transform2d(front_right_position, modules[0].getAngle())),
            robot_pose.transformBy(new Transform2d(front_left_position, modules[1].getAngle())),
            robot_pose.transformBy(new Transform2d(back_right_position, modules[2].getAngle())),
            robot_pose.transformBy(new Transform2d(back_left_position, modules[3].getAngle()))
            );
        
        FieldVisualizer.getField().getObject("Cameras").setPoses(SubSystems.vision.getAllRobotToCameraPoses(robot_pose));
    }

    public Pose2d getPose(){ return robot_pose; }
    public SwerveDriveKinematics getKinematics(){ return kinematics; }

    public void stopModules(){ 
        for(int i = 0; i < 4; i++) modules[i].stop(); 
    }
    public void xStopModules(){ 
        for(int i = 0; i < 4; i++) modules[i].xStop(); 
    }
    public void setModuleStates(SwerveModuleState[] desired_states){
        for(int i = 0; i < 4; i++) modules[i].runSetpoint(desired_states[i]);
        
        if(Robot.isSimulation()){
            ChassisSpeeds chassis_speeds = kinematics.toChassisSpeeds(desired_states);
            navx.updateSimulationAngle((
                Rotation2d.fromRadians(
                    MathUtil.clamp(chassis_speeds.omegaRadiansPerSecond, -TELEOP_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, TELEOP_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND) 
                        * Robot.defaultPeriodSecs)));
        }
    }
}