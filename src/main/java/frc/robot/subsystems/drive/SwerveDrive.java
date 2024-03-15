package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import static edu.wpi.first.units.Units.*;
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
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.Constants.SubSystems;
import frc.robot.subsystems.vision.VisionCamera.TimestampedVisionMeasurement;
import frc.robot.utils.FieldVisualizer;
import frc.robot.utils.Print;
import frc.robot.utils.TimerUtil;

public class SwerveDrive extends SubsystemBase {
    public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(15.1);
    public static final double TELEOP_DRIVE_MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(15.1); // ?
    public static final double DRIVE_BASE_RADIUS =
        Math.hypot(Units.inchesToMeters(29) / 2.0, Units.inchesToMeters(29) / 2.0);
    public static final double TELEOP_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = PHYSICAL_MAX_SPEED_METERS_PER_SECOND / DRIVE_BASE_RADIUS;
    
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

    private static final HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    Units.feetToMeters(15.1), // Max module speed, in m/s
                    CENTER_TO_MODULE, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
                    );

    private final NavX navx = Robot.isReal() ? new NavXReal(NAVX_PORT) : new NavXSim(NAVX_PORT);
    private final SwerveModule[] modules;
    private final SwerveDriveOdometry odometer;
    private final SwerveDrivePoseEstimator pose_estimator;

    private Pose2d robot_pose = RobotState.isBlueAlliance() ? new Pose2d(1.567501, 5.380708, Rotation2d.fromDegrees(180)) : new Pose2d(14, 5.380708, new Rotation2d());
    // private Pose2d robot_pose = new Pose2d(0.77, 6.58, Rotation2d.fromDegrees(-117.90));

    private final SysIdRoutine sysId;

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
        
        //https://pathplanner.dev/pplib-build-an-auto.html#configure-autobuilder
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotVelocity, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            config, //HolonomicPathFollowerConfig
            () -> { return RobotState.isRedAlliance(); // Should mirror path
            },
            this // Reference to this subsystem to set requirements
        );

        sysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null, null, null,
                    (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> {
                    for (int i = 0; i < 4; i++) {
                        modules[i].runCharacterization(voltage.in(Volts));
                    }
                    }, null,
                    this));

        Logger.recordOutput("PoseEstimation/VisionMeasurement", new Pose3d());
    }

    @Override public void periodic(){
        for (var module : modules) module.periodic();
        navx.periodic();
        var vision_measurements = SubSystems.vision.getAllVisionMeasurements();
        for (TimestampedVisionMeasurement vision_measurement : vision_measurements){
            if(!DriverStation.isAutonomousEnabled())
                pose_estimator.addVisionMeasurement(vision_measurement.pose.toPose2d(), vision_measurement.timestamp, vision_measurement.std_devs);
            
            Logger.recordOutput("PoseEstimation/VisionMeasurement", vision_measurement.pose);
            Logger.recordOutput("Vision/StdDevs-X", vision_measurement.std_devs.get(0, 0));
            Logger.recordOutput("Vision/StdDevs-Y", vision_measurement.std_devs.get(1, 0));
            Logger.recordOutput("Vision/StdDevs-Theta", vision_measurement.std_devs.get(2, 0));
        }
        odometer.update(getRotation2d(), getWheelPositions());
        pose_estimator.update(getRotation2d(), getWheelPositions());

        robot_pose = pose_estimator.getEstimatedPosition();

        FieldVisualizer.getField().setRobotPose(robot_pose);
        FieldVisualizer.getField().getObject("SwerveModules").setPoses(
            robot_pose.transformBy(new Transform2d(front_right_position, modules[0].getAngle())),
            robot_pose.transformBy(new Transform2d(front_left_position, modules[1].getAngle())),
            robot_pose.transformBy(new Transform2d(back_right_position, modules[2].getAngle())),
            robot_pose.transformBy(new Transform2d(back_left_position, modules[3].getAngle()))
            );
        
        FieldVisualizer.getField().getObject("Cameras").setPose(SubSystems.vision.getAllCameras().get(0).getCameraPoseOnRobot(robot_pose).toPose2d());

        Logger.recordOutput("Drive/Modules/States/Actual", getModuleStates());
        Logger.recordOutput("Drive/Modules/States/Sim", getSimModuleStates());
        Logger.recordOutput("Drive/Modules/Positions", getModulePositions());
        Logger.recordOutput("PoseEstimation/Odometer", odometer.getPoseMeters());
        Logger.recordOutput("PoseEstimation/NavX-Displacement", navx.getPose3d());
        Logger.recordOutput("PoseEstimation/NavX-Odometer", navx.getEstimatedPose());
        Logger.recordOutput("PoseEstimation/FullPoseEstimator", robot_pose);
        // Logger.recordOutput("DriverCamPose", 
            // new Pose3d(robot_pose).plus(SubSystems.vision.getCamera("DriverCam").getTransform()));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) { return sysId.quasistatic(direction); }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) { return sysId.dynamic(direction); }

    public void postEnableSetup(){
        navx.reset();
        navx.setStartPose(robot_pose);
        Pose2d held_pose = robot_pose;
        if(RobotState.isBlueAlliance()) Print.log("BLUE ALLIANCE");
        setRotationOffset(RobotState.isBlueAlliance() ? held_pose.getRotation() : held_pose.getRotation().plus(Rotation2d.fromDegrees(180)));
        resetPose(held_pose);
    }
    public void setRotationOffset(Rotation2d rotation){ navx.setFieldOrientedRotationOffset(rotation); }
    public Rotation2d getRotation2d(){ return navx.getRotation2d(); }
    public Rotation2d getRotationOffset(){ return navx.getFieldOrientedRotationOffset(); }
    public NavX getNavX(){ return navx; }

    private void resetPose(Pose2d pose){
        odometer.resetPosition(getRotation2d(), getWheelPositions(), pose);
        pose_estimator.resetPosition(getRotation2d(), getWheelPositions(), pose);
    }
    private ChassisSpeeds getRobotRelativeSpeeds(){ return kinematics.toChassisSpeeds(getModuleStates()); }
    public void driveRobotVelocity(ChassisSpeeds speeds){ 
        if(Robot.isReal()) setModuleStates(kinematics.toSwerveModuleStates(speeds.unaryMinus()));
        else setModuleStates(kinematics.toSwerveModuleStates(speeds));
    }

    private SwerveDriveWheelPositions getWheelPositions(){ return new SwerveDriveWheelPositions(getModulePositions()); }
    private SwerveModuleState[] getModuleStates(){
        return new SwerveModuleState[] {
                modules[0].getState(),
                modules[1].getState(),
                modules[2].getState(),
                modules[3].getState()};
    }
    private SwerveModuleState[] getSimModuleStates(){
        return new SwerveModuleState[] {
                modules[0].getSimModuleState(),
                modules[1].getSimModuleState(),
                modules[2].getSimModuleState(),
                modules[3].getSimModuleState()};
    }
    public SwerveModulePosition[] getModulePositions(){
        return new SwerveModulePosition[] {
                modules[0].getPosition(),
                modules[1].getPosition(),
                modules[2].getPosition(),
                modules[3].getPosition()};
    }
    public SwerveModule[] getModules(){ synchronized(modules){ return modules; } }
    public Pose2d getPose(){ return pose_estimator.getEstimatedPosition(); }
    public SwerveDriveKinematics getKinematics(){ return kinematics; }
    public SwerveDriveKinematicsConstraint getKinematicsConstraint(){ return kinematics_constrait; }

    public void stopModules(){ for(int i = 0; i < 4; i++) modules[i].stop(); }
    public void xStopModules(){ for(int i = 0; i < 4; i++) modules[i].xStop(); }
    public void setModuleStates(SwerveModuleState[] desired_states){
        SwerveDriveKinematics.desaturateWheelSpeeds(desired_states, PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
        SwerveModuleState[] optimized_states = new SwerveModuleState[4];
        for(int i = 0; i < 4; i++) optimized_states[i] = modules[i].runSetpoint(desired_states[i]);

        Logger.recordOutput("Drive/Modules/States/Desired", desired_states);
        Logger.recordOutput("Drive/Modules/States/DesiredOptimized", optimized_states);
    }
}