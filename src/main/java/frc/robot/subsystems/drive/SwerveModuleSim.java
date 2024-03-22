package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;

public class SwerveModuleSim extends SwerveModule {
    private final FlywheelSim drive_sim = new FlywheelSim(DCMotor.getNEO(1), 1 / DRIVE_MOTOR_GEAR_RATIO, 0.025);
    private final FlywheelSim turn_sim =  new FlywheelSim(DCMotor.getNEO(1), 1 / TURN_MOTOR_GEAR_RATIO, 0.004);

    private double drive_position_radians = 0.0;
    private double turn_relative_position_radians = 0.0;
    private double turn_absolute_position_radians = Math.random() * 2.0 * Math.PI;
    private double drive_applied_volts = 0.0;
    private double turn_applied_volts = 0.0;

    public SwerveModuleSim(String module_name, int index) { 
        super(module_name, index); 
    }

    @Override public void updateInputs() {
        drive_sim.update(Robot.defaultPeriodSecs);
        turn_sim.update(Robot.defaultPeriodSecs);

        double angleDiffRad = turn_sim.getAngularVelocityRadPerSec() * Robot.defaultPeriodSecs;
        turn_relative_position_radians += angleDiffRad;
        turn_absolute_position_radians += angleDiffRad;
        while (turn_absolute_position_radians < 0) {
            turn_absolute_position_radians += 2.0 * Math.PI;
        }
        while (turn_absolute_position_radians > 2.0 * Math.PI) {
            turn_absolute_position_radians -= 2.0 * Math.PI;
        }

        drive_position_radians += drive_sim.getAngularVelocityRadPerSec() * Robot.defaultPeriodSecs;
        inputs.drive_position_meters = drive_position_radians * (WHEEL_DIAMETER_METERS);
        inputs.drive_velocity_meters_per_second = drive_sim.getAngularVelocityRadPerSec() * (WHEEL_DIAMETER_METERS);
        inputs.drive_applied_volts = drive_applied_volts;
        inputs.drive_current_amps = new double[] {Math.abs(drive_sim.getCurrentDrawAmps())};
        inputs.drive_temp_celcius = new double[] {};

        inputs.turn_absolute_position_radians = turn_absolute_position_radians;
        inputs.turn_position_radians = turn_relative_position_radians;
        inputs.turn_velocity_radians_per_second = turn_sim.getAngularVelocityRadPerSec();
        inputs.turn_applied_volts = turn_applied_volts;

        inputs.sim_module_state = new SwerveModuleState(inputs.drive_velocity_meters_per_second, new Rotation2d(inputs.turn_position_radians));
    }

    @Override public void setDriveVoltage(double volts) {
        drive_applied_volts = MathUtil.clamp(volts, -12.0, 12.0);
        final double drive_applied_volts_fluc = Math.abs(drive_applied_volts) - drive_feedforward.ks - Math.random();
        drive_applied_volts = Math.copySign(drive_applied_volts_fluc < 0 ? 0 : drive_applied_volts_fluc, drive_applied_volts); 
        drive_sim.setInputVoltage(drive_applied_volts);
    }

    @Override public void setTurnVoltage(double volts) {
        turn_applied_volts = MathUtil.clamp(volts, -12.0, 12.0);
        turn_sim.setInputVoltage(turn_applied_volts);
    }

    @Override public void setDriveBrakeMode(boolean enable) {}
    @Override public void setTurnBrakeMode(boolean enable) {}

    @Override public void setTurn(double value) { setTurnVoltage(value * 12); }
}