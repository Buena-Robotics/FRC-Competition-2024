package frc.robot.utils;

import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;

public class NoteVisualizer {
    // private static final Translation3d BLUE_SPEAKER_POSE = new Translation3d(0.225, 5.55, 2.1);
    // private static final Translation3d RED_SPEAKER_POSE = new Translation3d(16.317, 5.55, 2.1);
    private static final double shot_speed_meters_per_seconds = 4.0; // Meters per sec

    public static Command shoot(final Supplier<Pose2d> robot_pose_supplier, final Supplier<Double> shooter_pitch_radians_supplier) {
        return new ScheduleCommand(
            Commands.defer(
                    () -> {
                    final Transform3d shooter_transform =
                        new Transform3d(
                            new Translation3d(Units.inchesToMeters(-14.5),0.0,Units.inchesToMeters(21.5)), 
                            new Rotation3d(0.0,-((Math.PI/2) - shooter_pitch_radians_supplier.get()),0.0));
                    final Pose3d shooter_start_pose = new Pose3d(robot_pose_supplier.get()).transformBy(shooter_transform);
                    final Pose3d shooter_end_pose = shooter_start_pose.plus(
                        new Transform3d(
                            new Translation3d(Units.feetToMeters(26), Units.feetToMeters(7) / 2, -1.45), 
                        new Rotation3d()));

                    final double duration = shooter_start_pose.getTranslation().getDistance(shooter_end_pose.getTranslation()) / shot_speed_meters_per_seconds;
                    final Timer timer = new Timer();
                    timer.start();
                    return Commands.run(
                            () -> {
                                Logger.recordOutput(
                                    "Shooter/NoteVisualizer",
                                    new Pose3d[] {
                                    shooter_start_pose.interpolate(shooter_end_pose, timer.get() / duration)
                                    });
                            })
                        .until(() -> timer.hasElapsed(duration))
                        .finallyDo(
                            () -> {
                                Logger.recordOutput("Shooter/NoteVisualizer", new Pose3d[] {});
                            });
                    },
                    Set.of())
                .ignoringDisable(true));
    }
}
