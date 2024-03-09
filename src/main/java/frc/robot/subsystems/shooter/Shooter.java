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
