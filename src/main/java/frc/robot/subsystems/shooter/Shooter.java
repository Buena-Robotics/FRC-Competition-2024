package frc.robot.subsystems.shooter;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.Constants.SubSystems;

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

        final double shooter_pitch_radians = (Math.PI/2) - SubSystems.climb.getShooterAngleRadians();
        
        final Pose2d robot_pose = SubSystems.swerve_drive.getPose();
        final double speaker_height = Units.feetToMeters(6 + (16/12.0)) - Units.inchesToMeters(21.5);
        final double distance_to_speaker = robot_pose.getTranslation().getDistance(FieldConstants.getSpeakerPoint().getTranslation());

        final double estimated_shooter_pitch = Math.atan2(speaker_height, distance_to_speaker);

        // if(inputs.holding_note_beam_broke)
            SubSystems.climb.runSetpoint(new Rotation2d((Math.PI/2) - estimated_shooter_pitch));

        final List<Pose3d> estimated_note_trajectory = new ArrayList<Pose3d>(2);
        estimated_note_trajectory.add(getShooterPose(estimated_shooter_pitch));
        estimated_note_trajectory.add(getShooterPose(estimated_shooter_pitch).transformBy(
            new Transform3d(
                new Translation3d(Math.sqrt( Math.pow(speaker_height, 2) + Math.pow(distance_to_speaker, 2) ), Math.sqrt(distance_to_speaker) / 1.9, 0), 
                new Rotation3d())));
        Pose3d[] estimated_note_trajectory_as_array = new Pose3d[estimated_note_trajectory.size()];
        estimated_note_trajectory_as_array = estimated_note_trajectory.toArray(estimated_note_trajectory_as_array);
        
        Logger.recordOutput("Shooter/ShooterPose", getShooterPose(shooter_pitch_radians));
        Logger.recordOutput("Shooter/EstimatedShooterPose", getShooterPose(estimated_shooter_pitch));
        Logger.recordOutput("Shooter/EstimatedShooterTrajectory", estimated_note_trajectory_as_array);
    }

    private Pose3d getShooterPose(double shooter_pitch_radians){
        final Transform3d robot_to_shooter = new Transform3d(new Translation3d(Units.inchesToMeters(-14.5),0.0,Units.inchesToMeters(21.5)), new Rotation3d(0.0,-shooter_pitch_radians,0.0));
        final Pose3d robot_pose = new Pose3d(SubSystems.swerve_drive.getPose());
        final Pose3d shooter_pose = robot_pose.plus(robot_to_shooter);
        return shooter_pose;
    }

    public boolean hasNote(){
        return inputs.holding_note_beam_broke;
    }

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
