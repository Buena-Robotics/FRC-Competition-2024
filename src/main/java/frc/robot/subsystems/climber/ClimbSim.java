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
    
    private final Mechanism2d winch_mechanism = new Mechanism2d(60, 60);
    private final Mechanism2d arm_mechanism = new Mechanism2d(60, 60);
    private final MechanismLigament2d winch_ligament = new MechanismLigament2d("winch_ligament", 5, 70, 4, new Color8Bit("#5050FF"));
    private final MechanismLigament2d arm_ligament = new MechanismLigament2d("arm_ligament", 20, 86, 4, new Color8Bit("#802020"));

    public ClimbSim(){
        super();
        winch_mechanism.getRoot("winch_root", 6, 6).append(winch_ligament);
        arm_mechanism.getRoot("arm_root", 5, 30).append(arm_ligament);
        SmartDashboard.putData("Climb/Mechanism", winch_mechanism);
        SmartDashboard.putData("Climb/Mechanism", arm_mechanism);
    }

    @Override public void updateInputs() {
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
        winch_ligament.setAngle(70 + -inputs.winch_rotations*360);
        arm_ligament.setAngle(Units.radiansToDegrees((Math.PI/2) - inputs.bore_absolute_position_radians));
        Logger.recordOutput("Climb/Mecha/Winch Mechansim", winch_mechanism);
        Logger.recordOutput("Climb/Mecha/Arm Mechanism", arm_mechanism);
    }    
}
