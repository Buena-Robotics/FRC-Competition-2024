package frc.robot.subsystems.climber;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubSystems;

public abstract class Climb extends SubsystemBase {
    @AutoLog public static class ClimbInputs {
        public double bore_absolute_position_raw = 0.0;
        public double bore_absolute_position_radians = 0.0;

        public boolean winch_top_limit_switch_triggered = false;
        public boolean winch_bottom_limit_switch_triggered = false;
        
        public double winch_position_radians = 0.0;
        public double winch_velocity_radians_per_second = 0.0;
        public double winch_velocity_rotations_per_second = 0.0;
        public double winch_rotations = 0.0;
        public double winch_applied_volts = 0.0;
        public double[] winch_current_amps = new double[] {};
        public double[] winch_temp_celcius = new double[] {};
    }
    public enum ArmPosition {
        DOWN(new Rotation2d(1.57079632679)), 
        SOURCE(new Rotation2d(1.11923695855)), 
        SPEAKER_CLOSE(new Rotation2d(0.67147204082)),
        SPEAKER_STAGE(new Rotation2d(0.861727450426)),
        UP(new Rotation2d(0));

        private Rotation2d rotation;
        private ArmPosition(Rotation2d rotation) { this.rotation = rotation; }
        public Rotation2d getRotation() { return this.rotation; };
    }
    
    protected static final double WINCH_ROPE_LENGTH_METERS = Units.inchesToMeters(17);
    protected static final double WINCH_MOTOR_GEAR_RATIO = 1 / 64.0;
    protected static final double WINCH_ENCODER_ROTATIONS_TO_RADIANS = Math.PI * 2;
    protected static final double WINCH_TOTAL_FULL_ROTATIONS = 1.853;
    
    protected ClimbInputsAutoLogged inputs = new ClimbInputsAutoLogged();
    
    private final Mechanism2d winch_mechanism = new Mechanism2d(60, 60);
    private final Mechanism2d arm_mechanism = new Mechanism2d(60, 60);
    private final MechanismLigament2d winch_ligament = new MechanismLigament2d("winch_ligament", 5, 70, 4, new Color8Bit("#5050FF"));
    private final MechanismLigament2d arm_ligament = new MechanismLigament2d("arm_ligament", 20, 86, 4, new Color8Bit("#802020"));

    public Climb(){
        winch_mechanism.getRoot("winch_root", 6, 6).append(winch_ligament);
        arm_mechanism.getRoot("arm_root", 5, 30).append(arm_ligament);
        SmartDashboard.putData("Climb/Mechanism", winch_mechanism);
        SmartDashboard.putData("Climb/Mechanism", arm_mechanism);
    }

    public abstract void updateInputs();
    public abstract void setWinchVoltage(double volts);

    @Override public void periodic() {
        updateInputs();
        Logger.processInputs("Climb", inputs);

        winch_ligament.setAngle(70 + -inputs.winch_rotations*360);
        arm_ligament.setAngle(Units.radiansToDegrees((Math.PI/2) - inputs.bore_absolute_position_radians));
        
        Logger.recordOutput("Climb/Mecha/Winch Mechansim", winch_mechanism);
        Logger.recordOutput("Climb/Mecha/Arm Mechanism", arm_mechanism);
    }

    public void runSetpoint(Rotation2d setpoint){
        final Rotation2d measurement = new Rotation2d(inputs.bore_absolute_position_radians);
        
        double voltage = 0.0;
        if(measurement.minus(setpoint).getRadians() < 0)
            voltage = Math.sqrt(measurement.unaryMinus().plus(setpoint).getDegrees()) + 1;
        else 
            voltage = -Math.sqrt(measurement.minus(setpoint).getDegrees()) - 1;
        voltage = MathUtil.clamp(voltage, -12, 12);

        if(inputs.winch_rotations < 4/64.0 && voltage < 0){
            setWinchVoltage(0);
            return;
        }
        else if(inputs.winch_rotations > 112/64.0 && voltage > 0){
            setWinchVoltage(0);
            return;
        }
        setWinchVoltage(voltage);
    }

    public Command moveArmTriggers(Supplier<Double> speed_function){
        return new Command() {
            { addRequirements(SubSystems.climb); }
            private final Supplier<Double> speed = speed_function;
            @Override public void execute() {
                double voltage = speed.get() * 12;
                voltage = MathUtil.clamp(voltage, -12, 12);
                if(inputs.winch_rotations < 4/64.0 && voltage < 0){
                    setWinchVoltage(0);
                    return;
                }
                else if(inputs.winch_rotations > 120/64.0 && voltage > 0){
                    setWinchVoltage(0);
                    return;
                }
                setWinchVoltage(speed.get() * 12);
            }
            @Override public void end(boolean interrupted) { setWinchVoltage(0); }
            @Override public boolean isFinished() { return false; }
        };
    }
    public Command moveArmToPosition(ArmPosition position){ return moveArmToPosition(position.getRotation()); }
    public Command moveArmToPosition(Rotation2d rotation) {
        return new Command() {
            private final Rotation2d setpoint = rotation;
            private Rotation2d measurement = new Rotation2d(inputs.bore_absolute_position_radians);
            private Rotation2d prev_measurement = new Rotation2d(inputs.bore_absolute_position_radians);
            private final int wrong_direction_period_counter_max = 20;
            private int wrong_direction_counter = 0;

            @Override public void initialize() {
                Logger.recordOutput("Climb/Preset/Setpoint", setpoint);
                addRequirements(SubSystems.climb);
            }

            @Override public void execute() {
                if(wrong_direction_counter % wrong_direction_period_counter_max == wrong_direction_period_counter_max-1) this.prev_measurement = measurement;
                runSetpoint(setpoint);
                this.measurement = new Rotation2d(inputs.bore_absolute_position_radians);
                Logger.recordOutput("Climb/Preset/Measurement", measurement);
            }
            @Override public void end(boolean interrupted) { 
                setWinchVoltage(0);
                Logger.recordOutput("Climb/Preset/Setpoint", new Rotation2d(-1.0));
                Logger.recordOutput("Climb/Preset/Measurement", new Rotation2d(-1.0));
            }
            @Override public boolean isFinished() {
                boolean wrong_direction = false;
                if(wrong_direction_counter++ % wrong_direction_period_counter_max == wrong_direction_period_counter_max-1)
                    wrong_direction = Math.abs(measurement.minus(setpoint).getRadians()) > Math.abs(prev_measurement.minus(setpoint).getRadians());
                if(wrong_direction) DriverStation.reportWarning("move arm to position cmd wrong direction", false);
                return wrong_direction
                    || Math.abs(setpoint.minus(measurement).getDegrees()) < 3;
            }
        };
    }
}
