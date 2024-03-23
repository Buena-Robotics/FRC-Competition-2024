package frc.robot.subsystems.climber;

import frc.robot.utils.ULogger;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubSystems;
import frc.robot.utils.Print;

public abstract class Climb extends SubsystemBase {
    @AutoLog public static class ClimbInputs {
        public boolean bore_encoder_connected = true;

        public double bore_absolute_position_raw = 0.0;
        public double bore_absolute_position_radians = 0.0;

        public boolean winch_top_limit_switch_triggered = false;
        public boolean winch_bottom_limit_switch_triggered = false;
        
        public double winch_position_radians = 0.0;
        public double winch_velocity_radians_per_second = 0.0;
        public double winch_velocity_rotations_per_second = 0.0;
        public double winch_rotations = 0.0;
        public double winch_applied_volts = 0.0;
    }
    public enum ArmPosition {
        DOWN(new Rotation2d(1.57079632679 + 0.101)), 
        SOURCE(new Rotation2d(1.11923695855 + 0.101)),
        SPEAKER_CLOSE(new Rotation2d(0.67147204082 + 0.101-0.03)),
        SPEAKER_STAGE(new Rotation2d(0.908)),        
        FAR(new Rotation2d(0.983)),

        UP(new Rotation2d());

        private Rotation2d rotation;
        private ArmPosition(Rotation2d rotation) { this.rotation = rotation; }
        public Rotation2d getRotation() { return this.rotation; };
    }
    
    protected static final double WINCH_ROPE_LENGTH_METERS = Units.inchesToMeters(17);
    protected static final double WINCH_MOTOR_GEAR_RATIO = 1 / 64.0;
    protected static final double WINCH_ENCODER_ROTATIONS_TO_RADIANS = Math.PI * 2;
    protected static final double WINCH_TOTAL_FULL_ROTATIONS = 1.853;
    
    protected ClimbInputsAutoLogged inputs = new ClimbInputsAutoLogged();

    private final Mechanism2d arm_mechanism = new Mechanism2d(Units.inchesToMeters(29), Units.inchesToMeters(29));
    private final MechanismLigament2d arm_ligament = new MechanismLigament2d("arm_ligament", Units.inchesToMeters(29), 86, 6, new Color8Bit("#802020"));

    public Climb(){
        arm_mechanism.getRoot("arm_root", Units.inchesToMeters(-2), 0)
            .append(new MechanismLigament2d("fixed", Units.inchesToMeters(21.5), 90))
            .append(arm_ligament);

        SmartDashboard.putData("Climb/ArmVisualization", arm_mechanism);
    }

    public abstract void updateInputs();
    public abstract void setWinchVoltage(double volts);

    @Override public void periodic() {
        updateInputs();
        Logger.processInputs("Climb", inputs);

        arm_ligament.setAngle(Units.radiansToDegrees(-inputs.bore_absolute_position_radians));

        ULogger.recordOutput("Climb/BoreRadians", inputs.bore_absolute_position_radians);
        ULogger.recordOutput("Climb/Mecha/Arm Mechanism", arm_mechanism);
        if(!inputs.bore_encoder_connected) Print.error("'Bore Encoder' Not Connected [Climb]");
    }

    public double getShooterAngleRadians(){ return inputs.bore_absolute_position_radians; }
    public Rotation2d getShooterAngleRotation(){ return new Rotation2d(inputs.bore_absolute_position_radians); }

    public boolean runSetpoint(Rotation2d setpoint){
        return runSetpoint(setpoint, 3);
    }

    public boolean runSetpoint(Rotation2d setpoint, double scalar){
        final Rotation2d measurement = new Rotation2d(inputs.bore_absolute_position_radians);
        
        double voltage = 0.0;
        if(measurement.minus(setpoint).getDegrees() < 0)
            voltage = Math.sqrt(measurement.unaryMinus().plus(setpoint).getDegrees());
        else 
            voltage = -Math.sqrt(measurement.minus(setpoint).getDegrees());
        voltage = MathUtil.clamp(voltage * scalar, -12, 12);

        if(inputs.winch_rotations < 4/64.0 && voltage < 0){
            setWinchVoltage(0);
            return false;
        }
        else if(inputs.winch_rotations > 112/64.0 && voltage > 0){
            setWinchVoltage(0);
            return false;
        }
        setWinchVoltage(voltage);
        return true;
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
            private boolean setpoint_good = true;

            { addRequirements(SubSystems.climb); }

            @Override public void initialize() {
                ULogger.recordOutput("Climb/Preset/Setpoint", setpoint);
                addRequirements(SubSystems.climb);
            }

            @Override public void execute() {
                setpoint_good = runSetpoint(setpoint);
                this.measurement = new Rotation2d(inputs.bore_absolute_position_radians);
                ULogger.recordOutput("Climb/Preset/Measurement", measurement);
            }
            @Override public void end(boolean interrupted) { 
                setWinchVoltage(0);
                ULogger.recordOutput("Climb/Preset/Setpoint", new Rotation2d(-1.0));
                ULogger.recordOutput("Climb/Preset/Measurement", new Rotation2d(-1.0));
            }
            @Override public boolean isFinished() {
                return Math.abs(setpoint.minus(measurement).getDegrees()) < 2 || !setpoint_good;
            }
        };
    }
}
