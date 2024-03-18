package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubSystems;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.climber.Climb;

public abstract class Shooter extends SubsystemBase {
    @AutoLog public static class ShooterInputs {
        public boolean shooter_ledge_beam_broke = false;
        public boolean holding_note_beam_broke = false;

        public double feed_velocity_rotations_per_second = 0.0;
        public double feed_applied_volts = 0.0;
        public double[] feed_current_amps = new double[] {};
        public double[] feed_temp_celcius = new double[] {};

        public double launch_velocity_rotations_per_second = 0.0;
        public double launch_applied_volts = 0.0;
        public double[] launch_current_amps = new double[] {};
        public double[] launch_temp_celcius = new double[] {};
    }

    public static final double LAUNCH_SPEED_MAX = 1;
    public static final double INTAKE_LAUNCH_SPEED = -1; 
    public static final double INTAKE_FEED_SPEED = -.2;

    protected ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

    public Shooter() {}

    protected abstract void updateInputs();
    public abstract void setFeedVoltage(double volts);
    public abstract void setLaunchVoltage(double volts);
    
    @Override public void periodic(){
        updateInputs();
        Logger.processInputs("Shooter", inputs);
    }

    public static Rotation2d estimatedShooterRotation(){
        final Transform3d robot_to_shooter = new Transform3d(new Translation3d(Units.inchesToMeters(-14.5),0.0,Units.inchesToMeters(21.5)), new Rotation3d());
        final Pose3d robot_pose = new Pose3d(SubSystems.swerve_drive.getPose());
        final Pose3d shooter_pose = robot_pose.plus(robot_to_shooter);
        
        final double distance_to_speaker = shooter_pose.toPose2d().getTranslation().getDistance(FieldConstants.getSpeakerPoint().getTranslation());

        // final double bound = (distance_to_speaker - 1.646722) / (3.450181 - 1.646722);
        final double bound = (distance_to_speaker - 1.646722) / (4.5 - 1.646722);
        final Rotation2d estimated_rotation = Climb.ArmPosition.SPEAKER_CLOSE.getRotation().interpolate(Climb.ArmPosition.FAR.getRotation(), bound);  

        return estimated_rotation;
    }

    public boolean hasNote(){ return inputs.holding_note_beam_broke; }

    public Command intakeCommand() {
        return this.startEnd(() -> {
            if(!RobotState.armHasNote()){
                setFeedVoltage(INTAKE_FEED_SPEED * 12);
                setLaunchVoltage(INTAKE_LAUNCH_SPEED * 12);
            }
        }, () -> { stop(); });
    }

    public void stop() {
        setFeedVoltage(0);
        setLaunchVoltage(0);
    }
}
