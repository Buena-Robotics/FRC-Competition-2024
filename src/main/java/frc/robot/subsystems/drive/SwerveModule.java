package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.RobotConfig.RobotMode;

public abstract class SwerveModule {
    @AutoLog public static class SwerveModuleInputs {
        public double drive_position_raw = 0.0;
        public double drive_velocity_raw = 0.0;
        public double drive_position_meters = 0.0;
        public double drive_velocity_meters_per_second = 0.0;
        public double drive_applied_volts = 0.0;
        public double[] drive_current_amps = new double[] {};
        public double[] drive_temp_celcius = new double[] {};

        public double turn_absolute_position_radians = 0.0;
        public double turn_position_radians = 0.0;
        public double turn_velocity_radians_per_second = 0.0;
        public double turn_applied_volts = 0.0;
        public double[] turn_current_amps = new double[] {};
        public double[] turn_temp_celcius = new double[] {};

        public SwerveModuleState sim_module_state = new SwerveModuleState();
    }

    protected static final double WHEEL_DIAMETER_METERS  = Units.inchesToMeters(4);
    protected static final double DRIVE_MOTOR_GEAR_RATIO = 1 / 6.75;
    protected static final double TURN_MOTOR_GEAR_RATIO  = 1 / 12.8;
    protected static final double DRIVE_ENCODER_ROTATION_TO_METERS       = DRIVE_MOTOR_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
    protected static final double TURN_ENCODER_ROTATION_TO_RADIANS       = TURN_MOTOR_GEAR_RATIO * 2 * Math.PI;
    protected static final double DRIVE_ENCODER_RPM_TO_METERS_PER_SECOND = DRIVE_ENCODER_ROTATION_TO_METERS / 60;
    protected static final double TURN_ENCODER_RPM_TO_RADIANS_PER_SECOND = TURN_ENCODER_ROTATION_TO_RADIANS / 60;
    private static final double SET_STATE_SPEED_METERS_PER_SECOND_DEADBAND = 0.001;

    private SimpleMotorFeedforward drive_feedforward;
    private final PIDController turn_feedback;
    protected final String module_name;
    protected final int index;
    protected SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();
    // protected SwerveModuleOutputs outputs = new SwerveModuleOutputs();

    public SwerveModule(String module_name, int index){
        this.module_name = module_name;
        this.index = index;

        if(RobotConfig.getRobotMode() == RobotMode.REAL){            
            switch (index) {
                case 0: this.drive_feedforward = new SimpleMotorFeedforward(0.033927, 2.597, 0.34768); break;
                case 1: this.drive_feedforward = new SimpleMotorFeedforward(0.019646, 2.5956, 0.39179); break;
                case 2: this.drive_feedforward = new SimpleMotorFeedforward(0.010468, 2.7216, 0.40463); break;
                case 3: this.drive_feedforward = new SimpleMotorFeedforward(0.002596, 2.5386, 0.39606); break;
                default: this.drive_feedforward = new SimpleMotorFeedforward(0, 0, 0); break;
            }
        }
        else {
            this.drive_feedforward = new SimpleMotorFeedforward(0, 1.35);
        }
        this.turn_feedback  = new PIDController(0.7, 0.0, 0.0, Robot.defaultPeriodSecs);
        
        this.turn_feedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    public abstract void updateInputs();
    public abstract void setTurn(double value);
    public abstract void setDriveVoltage(double volts);
    public abstract void setTurnVoltage(double volts);
    public abstract void setDriveBrakeMode(boolean enable);
    public abstract void setTurnBrakeMode(boolean enable);

    public void periodic() {
        updateInputs();
        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);
    }

    public SwerveModuleState runSetpoint(SwerveModuleState state) {
        if(Math.abs(state.speedMetersPerSecond) < SET_STATE_SPEED_METERS_PER_SECOND_DEADBAND) {
            stop();
            return new SwerveModuleState();
        }
        SwerveModuleState optimized_state = SwerveModuleState.optimize(state, getAngle());

        setTurn( turn_feedback.calculate(inputs.turn_position_radians, optimized_state.angle.getRadians()));   

        optimized_state.speedMetersPerSecond *= Math.cos(turn_feedback.getPositionError());

        setDriveVoltage(drive_feedforward.calculate(optimized_state.speedMetersPerSecond));

        return optimized_state;
    }

    public void runCharacterization(double volts) {
        setTurnVoltage( turn_feedback.calculate(inputs.turn_position_radians, 0));  
        // Open loop drive control
        setDriveVoltage(volts);
    }

    public void stop() {
        setTurnVoltage(0.0);
        setDriveVoltage(0.0);
    }

    public void xStop(){
        setDriveVoltage(0.0);
        runSetpoint(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public SwerveModuleState getSimModuleState(){ return inputs.sim_module_state; }
    public Rotation2d getAngle() { return new Rotation2d(inputs.turn_position_radians); }
    public double getPositionMeters() { return inputs.drive_position_meters; }
    public double getVelocityMetersPerSec() { return inputs.drive_velocity_meters_per_second; }
    public SwerveModulePosition getPosition() { return new SwerveModulePosition(getPositionMeters(), getAngle()); }
    public SwerveModuleState getState() { return new SwerveModuleState(getVelocityMetersPerSec(), getAngle()); }
}