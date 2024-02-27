package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterSim extends Shooter {
    private final FlywheelSim intake_sim = new FlywheelSim(DCMotor.getNEO(1), 1, 0.004);
    private final FlywheelSim outtake_sim = new FlywheelSim(DCMotor.getNEO(1), 1, 0.004);
}