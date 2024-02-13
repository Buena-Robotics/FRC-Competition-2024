// Swerve Drive Code Learned From: https://www.youtube.com/watch?v=0Xi9yb1IMyA&t=284s FRC 0 to Autonomous: #6 Swerve Drive Auto

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

public abstract class SwerveModuleIO implements Sendable {
    protected static final double WHEEL_DIAMETER_METERS  = Units.inchesToMeters(4);
    protected static final double DRIVE_MOTOR_GEAR_RATIO = 1 / 6.75;
    protected static final double TURN_MOTOR_GEAR_RATIO  = 1 / 12.8;
    protected static final double PID_TURN_CONTROLLER_P = 0.5;
    protected static final double DRIVE_ENCODER_ROTATION_TO_METERS       = DRIVE_MOTOR_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
    protected static final double DRIVE_ENCODER_RPM_TO_METERS_PER_SECOND = DRIVE_ENCODER_ROTATION_TO_METERS / 60;
    protected static final double TURN_ENCODER_ROTATION_TO_RADIANS       = TURN_MOTOR_GEAR_RATIO * 2 * Math.PI; 
    protected static final double TURN_ENCODER_RPM_TO_RADIANS_PER_SECOND = TURN_ENCODER_ROTATION_TO_RADIANS / 60; 
    protected static final double SET_STATE_SPEED_METERS_PER_SECOND_DEADBAND = 0.001;

    protected final AnalogInput absolute_encoder;
    protected final double absolute_encoder_offset_radians;
    protected final PIDController turn_controller;

    protected SwerveModuleIO(int absolute_encoder_id, double absolute_encoder_offset_radians){
        this.turn_controller = new PIDController(PID_TURN_CONTROLLER_P, 0, 0);
        this.turn_controller.enableContinuousInput(-Math.PI, Math.PI);

        this.absolute_encoder = new AnalogInput(absolute_encoder_id);
        this.absolute_encoder_offset_radians = absolute_encoder_offset_radians;
    }

    public abstract double getDrivePosition();
    public abstract double getTurnPosition();

    public abstract double getDriveVelocity();
    public abstract double getTurnVelocity();

    public abstract void resetEncoders();
    public abstract void resetPosition();
    
    public abstract void stop();
    public abstract void setDesiredState(SwerveModuleState state);
    
    public double getAbsoluteEncoderRadians(){ 
        double angle = absolute_encoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        return angle - absolute_encoder_offset_radians;
    }

    public Rotation2d getRotation2d() { return new Rotation2d(getTurnPosition()); }
    public double getRotationDegrees(){ return getRotation2d().getDegrees(); }
    public double getRotationRadians(){ return getRotation2d().getRadians(); }

    public SwerveModuleState getState() { return new SwerveModuleState(getDriveVelocity(), getRotation2d()); }
    public SwerveModulePosition getPosition() { return new SwerveModulePosition(getDrivePosition(), getRotation2d()); }

    @Override public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("Swerve Drive/Swerve Module");
        builder.publishConstInteger("Channel", absolute_encoder.getChannel());
        builder.addDoubleProperty("Absolute Position", this::getAbsoluteEncoderRadians, null);
        builder.addDoubleProperty("Drive Position", this::getDrivePosition, null);
        builder.addDoubleProperty("Drive Velocity", this::getDriveVelocity, null);
        builder.addDoubleProperty("Turn Position", this::getTurnPosition, null);
        builder.addDoubleProperty("Turn Velocity", this::getTurnVelocity, null);
    }
}
