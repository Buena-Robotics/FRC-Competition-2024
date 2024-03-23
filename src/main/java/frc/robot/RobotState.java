package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.SubSystems;

public final class RobotState {
    public static boolean isRedAlliance(){
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }
    public static boolean isBlueAlliance(){ return !isRedAlliance(); }
    public static boolean shooterHasNote(){ return SubSystems.shooter.hasNote(); }
    public static boolean armHasNote(){ return SubSystems.note_arm.hasNote(); }
}
