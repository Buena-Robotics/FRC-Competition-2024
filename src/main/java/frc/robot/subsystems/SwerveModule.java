// Swerve Drive Code Learned From: https://www.youtube.com/watch?v=0Xi9yb1IMyA&t=284s FRC 0 to Autonomous: #6 Swerve Drive Auto

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveDrive;

public class SwerveModule implements Sendable {
    private static final double WHEEL_DIAMETER_METERS  = Units.inchesToMeters(4); // Not measured accurately
    private static final double DRIVE_MOTOR_GEAR_RATIO = 1 / 6.75;
    private static final double TURN_MOTOR_GEAR_RATIO  = 1 / 12.8;

    private static final double PID_TURN_CONTROLLER_P = 0.5;

    private static final double DRIVE_ENCODER_ROTATION_TO_METERS       = DRIVE_MOTOR_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
    private static final double DRIVE_ENCODER_RPM_TO_METERS_PER_SECOND = DRIVE_ENCODER_ROTATION_TO_METERS / 60;

    private static final double TURN_ENCODER_ROTATION_TO_RADIANS       = TURN_MOTOR_GEAR_RATIO * 2 * Math.PI; 
    private static final double TURN_ENCODER_RPM_TO_RADIANS_PER_SECOND = TURN_ENCODER_ROTATION_TO_RADIANS / 60; 

    private static final double SET_STATE_SPEED_METERS_PER_SECOND_DEADBAND = 0.001;

    private final CANSparkMax drive_motor;
    private final CANSparkMax turn_motor;

    private final RelativeEncoder drive_encoder;
    private final RelativeEncoder turn_encoder;

    private final PIDController turn_controller;

    private final AnalogInput absolute_encoder;
    private final boolean absolute_encoder_reversed;
    private final double absolute_encoder_offset_radians;

    public SwerveModule(int drive_motor_id, int turn_motor_id, boolean drive_motor_reversed, boolean turn_motor_reversed, int absolute_encoder_id, double absolute_encoder_offset_radians, boolean absolute_encoder_reversed) {
        this.drive_motor = new CANSparkMax(drive_motor_id, MotorType.kBrushless);
        this.turn_motor = new CANSparkMax(turn_motor_id, MotorType.kBrushless);

        this.drive_motor.setInverted(drive_motor_reversed);
        this.turn_motor.setInverted(turn_motor_reversed);

        this.drive_encoder = drive_motor.getEncoder();
        this.drive_encoder.setPositionConversionFactor(DRIVE_ENCODER_ROTATION_TO_METERS);
        this.drive_encoder.setVelocityConversionFactor(DRIVE_ENCODER_RPM_TO_METERS_PER_SECOND);
        
        this.turn_encoder = turn_motor.getEncoder();
        this.turn_encoder.setPositionConversionFactor(TURN_ENCODER_ROTATION_TO_RADIANS);
        this.turn_encoder.setVelocityConversionFactor(TURN_ENCODER_RPM_TO_RADIANS_PER_SECOND);

        this.turn_controller = new PIDController(PID_TURN_CONTROLLER_P, 0, 0);
        this.turn_controller.enableContinuousInput(-Math.PI, Math.PI);

        this.absolute_encoder = new AnalogInput(absolute_encoder_id);
        this.absolute_encoder_reversed = absolute_encoder_reversed;
        this.absolute_encoder_offset_radians = absolute_encoder_offset_radians;

        resetEncoders();
    }

    public double getDrivePosition(){ return drive_encoder.getPosition(); }
    public double getTurnPosition() { return turn_encoder.getPosition(); }

    public double getDriveVelocity() { return drive_encoder.getVelocity(); }
    public double getTurnVelocity()  { return  turn_encoder.getVelocity(); }

    public double getAbsoluteEncoderRadians(){ 
        double angle = absolute_encoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absolute_encoder_offset_radians;
        return absolute_encoder_reversed ? -angle : angle;
    }

    public void resetEncoders(){
        drive_encoder.setPosition(0);
        turn_encoder.setPosition(getAbsoluteEncoderRadians());
    }

    public Rotation2d getRotation2d(){ return new Rotation2d(getTurnPosition()); }
    public double getRotationDegrees() { return getRotation2d().getDegrees(); }
    public double getRotationRadians() { return getRotation2d().getRadians(); }

    public void resetPosition(){drive_encoder.setPosition(0);}

    public SwerveModuleState getState() { return new SwerveModuleState(getDriveVelocity(), getRotation2d()); }
    public SwerveModulePosition getPosition() { return new SwerveModulePosition(getDrivePosition(), getRotation2d()); }
    
    public void setDesiredState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond) < SET_STATE_SPEED_METERS_PER_SECOND_DEADBAND) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize( state, getState().angle );
        drive_motor.set( state.speedMetersPerSecond / SwerveDrive.PHYSICAL_MAX_SPEED_METERS_PER_SECOND );
        turn_motor.set( turn_controller.calculate( getTurnPosition(), state.angle.getRadians() ) );
    }

    public void stop(){
        drive_motor.set(0);
        turn_motor.set(0);
    }

    @Override public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("Swerve Module");
        builder.publishConstInteger("Channel", absolute_encoder.getChannel());
        builder.addDoubleProperty("Absolute Encoder Value Rad", this::getAbsoluteEncoderRadians, null);
        builder.addDoubleProperty("Drive Speed Meters|Sec", this::getDriveVelocity, null);
        builder.addDoubleProperty("Turn Speed Meters|Sec", this::getTurnVelocity, null);
        builder.addDoubleProperty("Rotation Rad", this::getRotationRadians, null);
    }
}
