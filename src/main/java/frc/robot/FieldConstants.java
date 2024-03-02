package frc.robot;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class FieldConstants {
    public static class AimAssistTarget{
        public final Pose2d pose; 
        public final Supplier<Boolean> condition;
        public AimAssistTarget(Pose2d pose, Supplier<Boolean> condition){
            this.pose = pose; 
            this.condition = condition; 
        }
    } 

    // private static final double NOTE_SIZE    = Units.inchesToMeters(13);
    // private static final double ROBOT_LENGTH = Units.inchesToMeters(26);
    // private static final double ROBOT_WIDTH  = Units.inchesToMeters(26);
    // private static final Pose2d
    //     NOTE_BLUE_TOP             = new Pose2d(2.887, 7.010, new Rotation2d()),
    //     NOTE_BLUE_CENTER          = new Pose2d(2.887, 5.560, new Rotation2d()),
    //     NOTE_BLUE_BOTTOM          = new Pose2d(2.887, 4.108, new Rotation2d()),
    //     NOTE_CENTER_TOP           = new Pose2d(8.270, 7.470, new Rotation2d()),
    //     NOTE_CENTER_CENTER_TOP    = new Pose2d(8.270, 5.790, new Rotation2d()),
    //     NOTE_CENTER_CENTER        = new Pose2d(8.270, 4.108, new Rotation2d()),
    //     NOTE_CENTER_CENTER_BOTTOM = new Pose2d(8.270, 2.430, new Rotation2d()),
    //     NOTE_CENTER_BOTTOM        = new Pose2d(8.270, 0.750, new Rotation2d()),
    //     NOTE_RED_TOP              = new Pose2d(13.65, 7.010, new Rotation2d()),
    //     NOTE_RED_CENTER           = new Pose2d(13.65, 5.560, new Rotation2d()),
    //     NOTE_RED_BOTTOM           = new Pose2d(13.65, 4.108, new Rotation2d());
    private static final Pose2d BLUE_SPEAKER_TARGET = new Pose2d(0.000000, 5.547868, new Rotation2d()); 
    private static final Pose2d RED_SPEAKER_TARGET = new Pose2d(16.541748, 5.547868, new Rotation2d());
    public static final AimAssistTarget[] aim_assist_targets = {
        new AimAssistTarget(new Pose2d(1.835, 7.798, Rotation2d.fromDegrees(90)), null), // Blue Amp
        new AimAssistTarget(new Pose2d(14.70, 7.798, Rotation2d.fromDegrees(90)), null) // Red Amp
    };

    public static boolean isRedAlliance(){
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }
    public static boolean isBlueAlliance(){ return RobotConfig.FORCE_BLUE_ALLIANCE || !isRedAlliance(); }
    public static Pose2d getSpeakerPoint(){ return isBlueAlliance() ? BLUE_SPEAKER_TARGET : RED_SPEAKER_TARGET; }

    public static Pair<AimAssistTarget, Double> getBestAimAssistTarget(Pose2d robot_pose){
        AimAssistTarget best_target = null;
        double best_distance = 64;
        for(AimAssistTarget target : aim_assist_targets){
            if(target.condition == null || target.condition.get()){
                double distance = robot_pose.getTranslation().getDistance(target.pose.getTranslation());
                if(distance < best_distance) {
                    best_target = target;
                    best_distance = distance;
                }
            }
        }
        return new Pair<FieldConstants.AimAssistTarget,Double>(best_target, best_distance);
    }
}