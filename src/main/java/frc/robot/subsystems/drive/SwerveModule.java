package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.TunableNumber;
import frc.robot.Robot;

public abstract class SwerveModule {
    @AutoLog public static class SwerveModuleInputs {
        public double drive_position_radians = 0.0;
        public double drive_velocity_radians_per_second = 0.0;
        public double drive_applied_volts = 0.0;
        public double[] drive_current_amps = new double[] {};
        public double[] drive_temp_celcius = new double[] {};

        public double turn_absolute_position_radians = 0.0;
        public double turn_position_radians = 0.0;
        public double turn_velocity_radians_per_second = 0.0;
        public double turn_applied_volts = 0.0;
        public double[] turn_current_amps = new double[] {};
        public double[] turn_temp_celcius = new double[] {};
    }

    protected static final double WHEEL_DIAMETER_METERS  = Units.inchesToMeters(4);
    protected static final double WHEEL_RADIUS_METERS  = WHEEL_DIAMETER_METERS / 2;
    protected static final double DRIVE_MOTOR_GEAR_RATIO = 1 / 6.75;
    protected static final double TURN_MOTOR_GEAR_RATIO  = 1 / 12.8;
    protected static final double DRIVE_ENCODER_ROTATION_TO_METERS       = DRIVE_MOTOR_GEAR_RATIO * WHEEL_DIAMETER_METERS * Math.PI;
    protected static final double DRIVE_ENCODER_RPM_TO_METERS_PER_SECOND = DRIVE_ENCODER_ROTATION_TO_METERS / 60;
    protected static final double TURN_ENCODER_ROTATION_TO_RADIANS       = TURN_MOTOR_GEAR_RATIO * 2 * Math.PI; 
    protected static final double TURN_ENCODER_RPM_TO_RADIANS_PER_SECOND = TURN_ENCODER_ROTATION_TO_RADIANS / 60; 

    private static final TunableNumber driveKp = new TunableNumber("Drive/Module/DriveKp", 1.0);
    private static final TunableNumber driveKd = new TunableNumber("Drive/Module/DriveKd");
    private static final TunableNumber driveKs = new TunableNumber("Drive/Module/DriveKs", 0.5);
    private static final TunableNumber driveKv = new TunableNumber("Drive/Module/DriveKv", 1.0);
    private static final TunableNumber turnKp = new TunableNumber("Drive/Module/TurnKp", 5.0);
    private static final TunableNumber turnKd = new TunableNumber("Drive/Module/TurnKd");
    
    private SimpleMotorFeedforward drive_feedforward;
    private final PIDController drive_feedback;
    private final PIDController turn_feedback;
    
    protected final String module_name;
    protected final int index;
    protected SwerveModuleInputsAutoLogged inputs;

    public SwerveModule(String module_name, int index){
        this.inputs = new SwerveModuleInputsAutoLogged();
        this.module_name = module_name;
        this.index = index;

        this.drive_feedforward = new SimpleMotorFeedforward(driveKs.get(), driveKv.get());
        this.drive_feedback = new PIDController(driveKp.get(), 0.0, driveKd.get(), Robot.defaultPeriodSecs);
        this.turn_feedback  = new PIDController(turnKp.get(), 0.0, turnKd.get(), Robot.defaultPeriodSecs);
        
        this.turn_feedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    public abstract void updateInputs();
    public abstract void setDriveVoltage(double volts);
    public abstract void setTurnVoltage(double volts);
    public abstract void setDriveBrakeMode(boolean enable);
    public abstract void setTurnBrakeMode(boolean enable);

    public void periodic() {
        updateInputs();
        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

        // Update controllers if tunable numbers have changed
        drive_feedback.setPID(driveKp.get(), 0.0, driveKd.get());
        turn_feedback.setPID(turnKp.get(), 0.0, turnKd.get());
        drive_feedforward = new SimpleMotorFeedforward(driveKs.get(), driveKv.get());
    }

    public SwerveModuleState runSetpoint(SwerveModuleState state) {
        SwerveModuleState optimized_state = SwerveModuleState.optimize(state, getAngle());

        setTurnVoltage( 
            turn_feedback.calculate(getAngle().getRadians(), optimized_state.angle.getRadians()) );

        optimized_state.speedMetersPerSecond *= Math.cos(turn_feedback.getPositionError());

        double velocity_radians_per_second = optimized_state.speedMetersPerSecond / WHEEL_RADIUS_METERS;
        setDriveVoltage(
            drive_feedforward.calculate(velocity_radians_per_second) 
                + drive_feedback.calculate(inputs.drive_velocity_radians_per_second, velocity_radians_per_second));
        return optimized_state;
    }

    public void stop() {
        setTurnVoltage(0.0);
        setDriveVoltage(0.0);
    }

    public void xStop(){
        setDriveVoltage(0.0);
        runSetpoint(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public Rotation2d getAngle() { return new Rotation2d(inputs.turn_position_radians); }
    public double getPositionMeters() { return inputs.drive_position_radians * DRIVE_MOTOR_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS; }
    public double getVelocityMetersPerSec() { return inputs.drive_velocity_radians_per_second * WHEEL_RADIUS_METERS; }
    public SwerveModulePosition getPosition() { return new SwerveModulePosition(getPositionMeters(), getAngle()); }
    public SwerveModuleState getState() { return new SwerveModuleState(getVelocityMetersPerSec(), getAngle()); }
}