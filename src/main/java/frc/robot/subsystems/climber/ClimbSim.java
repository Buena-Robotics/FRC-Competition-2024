package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Robot;

public class ClimbSim extends Climb {
    private final FlywheelSim winch_sim = new FlywheelSim(DCMotor.getNEO(1), 1 / WINCH_MOTOR_GEAR_RATIO, 0.00025);
    private double winch_applied_volts = 0;

    public ClimbSim(){
        super();
    }

    @Override public void updateInputs() {
        //TODO: Make Simulated limit switches :3
        inputs.winch_top_limit_switch_triggered = false;
        inputs.winch_bottom_limit_switch_triggered = false;

        inputs.winch_position_radians += (winch_sim.getAngularVelocityRadPerSec() * Robot.defaultPeriodSecs);
        inputs.winch_velocity_radians_per_second = winch_sim.getAngularVelocityRadPerSec();
        inputs.winch_rotations = inputs.winch_position_radians / (Math.PI * 2);
        inputs.winch_velocity_rotations_per_second = winch_sim.getAngularVelocityRPM() / 60;
        inputs.winch_applied_volts = winch_applied_volts;
        inputs.winch_current_amps = new double[] {winch_sim.getCurrentDrawAmps()};
        inputs.winch_temp_celcius = new double[] {};
        inputs.bore_absolute_position_radians =  (inputs.winch_rotations/WINCH_TOTAL_FULL_ROTATIONS) * (Math.PI/2);
    }
    @Override public void setWinchVoltage(double volts) {
        winch_applied_volts = MathUtil.clamp(volts, -12.0, 12.0);
        winch_sim.setInputVoltage(winch_applied_volts);
    }

    @Override public void periodic() {
        super.periodic();
        winch_sim.update(Robot.defaultPeriodSecs);
    }    
}
