package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    private Mechanism2d mechanism = new Mechanism2d(60, 60);
    private Rotation2d shooter_arm_rotation = Rotation2d.fromDegrees(95);
    private MechanismLigament2d shooter_arm_joint = new MechanismLigament2d("arm", 35, shooter_arm_rotation.getDegrees(), 4, new Color8Bit("#802020"));

    public Climb(){
        mechanism.getRoot("root", 10, 10).append(shooter_arm_joint);
        SmartDashboard.putData("Climb", mechanism);
    }

    

    @Override public void periodic() {
        shooter_arm_rotation = shooter_arm_rotation.plus(Rotation2d.fromDegrees(Constants.IO.controller.getLeftTriggerAxis()));
        shooter_arm_rotation = shooter_arm_rotation.minus(Rotation2d.fromDegrees(Constants.IO.controller.getRightTriggerAxis()));

        shooter_arm_rotation = Rotation2d.fromDegrees(MathUtil.clamp(shooter_arm_rotation.getDegrees(), 0, 95));

        shooter_arm_rotation = shooter_arm_rotation.plus(Rotation2d.fromDegrees(0.6));

        shooter_arm_joint.setAngle(shooter_arm_rotation);
    }    
}
