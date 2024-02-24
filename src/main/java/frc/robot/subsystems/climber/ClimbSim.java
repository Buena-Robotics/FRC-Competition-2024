package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Robot;

public class ClimbSim extends Climb{
    // private final FlywheelSim winch_sim = new FlywheelSim(DCMotor.getNEO(1), 1 / WINCH_ENCODER_GEAR_RATIO, 0.004);
    private final DCMotorSim winch_sim = new DCMotorSim(DCMotor.getNEO(1), 1 / WINCH_ENCODER_GEAR_RATIO, 0.004);
    
    private Mechanism2d mechanism = new Mechanism2d(60, 60);
    private Rotation2d shooter_arm_rotation = Rotation2d.fromDegrees(95);
    private MechanismLigament2d shooter_arm_joint = new MechanismLigament2d("arm", 35, shooter_arm_rotation.getDegrees(), 4, new Color8Bit("#802020"));

    public ClimbSim(){
        mechanism.getRoot("root", 10, 10).append(shooter_arm_joint);
        SmartDashboard.putData("Climb", mechanism);
    }

    @Override public void periodic() {
        winch_sim.update(Robot.defaultPeriodSecs);

        double voltage;
        if(Constants.IO.controller.getLeftTriggerAxis() > 0.01)
            voltage = Constants.IO.controller.getLeftTriggerAxis();
        else 
            voltage = -Constants.IO.controller.getRightTriggerAxis();

        voltage *= 12;

        winch_sim.setInputVoltage(voltage);
        double velocity = winch_sim.getAngularVelocityRadPerSec() * Robot.defaultPeriodSecs;
        SmartDashboard.putNumber("Climb/AngVelocity", winch_sim.getAngularVelocityRadPerSec());
        SmartDashboard.putNumber("Climb/PosRad", winch_sim.getAngularPositionRad());
        SmartDashboard.putNumber("Climb/PosRot", winch_sim.getAngularPositionRotations());
        SmartDashboard.putNumber("Climb/AngRPM", winch_sim.getAngularVelocityRPM());
        SmartDashboard.putNumber("Climb/AngRot", winch_sim.getAngularVelocityRPM() / 60 / (Math.PI * 2) * Robot.defaultPeriodSecs);



        shooter_arm_rotation = shooter_arm_rotation.plus(Rotation2d.fromRadians(winch_sim.getAngularVelocityRPM() / 60 / (Math.PI * 2) * Robot.defaultPeriodSecs * 10 ));
        // shooter_arm_rotation = shooter_arm_rotation.minus(Rotation2d.fromDegrees(Constants.IO.controller.getRightTriggerAxis() * 12));

        shooter_arm_rotation = shooter_arm_rotation.plus(Rotation2d.fromDegrees(0.001));
        shooter_arm_rotation = Rotation2d.fromDegrees(MathUtil.clamp(shooter_arm_rotation.getDegrees(), 0, 95));

        shooter_arm_joint.setAngle(shooter_arm_rotation);
    }    
}
