package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterSim extends Shooter {
    private final FlywheelSim feed_sim = new FlywheelSim(DCMotor.getNEO(1), 1, 0.004);
    private final FlywheelSim launch_sim = new FlywheelSim(DCMotor.getNEO(1), 1, 0.004);

    private double feed_applied_volts = 0.0, launch_applied_volts = 0.0;

    @Override public void updateInputs(){
        //use current pose + climb angle
        inputs.estimated_note_trajectory = new Pose3d[] {};

        inputs.shooter_ledge_beam_broke = false;
        inputs.holding_note_beam_broke = false;

        inputs.feed_velocity_rotations_per_second = feed_sim.getAngularVelocityRadPerSec();
        inputs.feed_applied_volts = feed_applied_volts;
        inputs.feed_current_amps = new double[] {};
        inputs.feed_temp_celcius = new double[] {};

        inputs.launch_velocity_rotations_per_second = launch_sim.getAngularVelocityRadPerSec();
        inputs.launch_applied_volts = launch_applied_volts = 0.0;
        inputs.launch_current_amps = new double[] {};
        inputs.launch_temp_celcius = new double[] {};
    }
    @Override public void setFeedVoltage(double volts){ 
        feed_applied_volts = MathUtil.clamp(volts, -12.0, 12.0);
        feed_sim.setInputVoltage(feed_applied_volts); 
    }
    @Override public void setLaunchVoltage(double volts){ 
        launch_applied_volts = MathUtil.clamp(volts, -12.0, 12.0);
        launch_sim.setInputVoltage(launch_applied_volts); 
    }
}