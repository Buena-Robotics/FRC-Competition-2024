package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class FieldConstants {
    private static final Pose2d BLUE_SPEAKER_TARGET = new Pose2d(0.000000, 5.547868, new Rotation2d()); 
    private static final Pose2d RED_SPEAKER_TARGET = new Pose2d(16.541748, 5.547868, new Rotation2d());
    public static Pose2d getSpeakerPoint(){ return RobotState.isBlueAlliance() ? BLUE_SPEAKER_TARGET : RED_SPEAKER_TARGET; }

    private static final Pose2d BLUE_SPEAKER_UPPER_PF = new Pose2d(0.804817, 6.673432, Rotation2d.fromDegrees(-122.781925));
    private static final Pose2d BLUE_SPEAKER_CENTER_PF = new Pose2d(1.330035, 5.765160, Rotation2d.fromDegrees(178.161603)); 
    private static final Pose2d BLUE_SPEAKER_LOWER_PF = new Pose2d(0.942494, 4.566208, Rotation2d.fromDegrees(119.675687)); 
    public static Pose2d getSpeakerUpperPathfindPose() { return BLUE_SPEAKER_UPPER_PF; }
    public static Pose2d getSpeakerCenterPathfindPose(){ return BLUE_SPEAKER_CENTER_PF; }
    public static Pose2d getSpeakerLowerPathfindPose() { return BLUE_SPEAKER_LOWER_PF; }

    private static final Pose2d BLUE_AMP_PF = new Pose2d(1.852956, 7.793584, Rotation2d.fromDegrees(91.831354));
    public static Pose2d getAmpPathfindPose(){ return BLUE_AMP_PF; }

    private static final Pose2d BLUE_SOURCE_LEFT_PF = new Pose2d(15.992217, 1.197372, Rotation2d.fromDegrees(-60));
    private static final Pose2d BLUE_SOURCE_CENTER_PF = new Pose2d(15.559432, 0.960882, Rotation2d.fromDegrees(-60));
    private static final Pose2d BLUE_SOURCE_RIGHT_PF = new Pose2d(14.928739, 0.606973, Rotation2d.fromDegrees(-60));
    public static Pose2d getSourceLeftPathfindPose()  { return BLUE_SOURCE_LEFT_PF; }
    public static Pose2d getSourceCenterPathfindPose(){ return BLUE_SOURCE_CENTER_PF; }
    public static Pose2d getSourceRightPathfindPose() { return BLUE_SOURCE_RIGHT_PF; }

    // private static final double NOTE_SIZE    = Units.inchesToMeters(13);
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
}