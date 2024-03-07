package frc.robot.subsystems.drive;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.SubSystems;

public class NavXSim extends NavX {
    public NavXSim(SerialPort.Port serial_port_id){
        super(serial_port_id);
    }

    private Pose2d last_sim_odometry = new Pose2d();
    private Pose2d sim_odometry = new Pose2d();
    double[] last_module_positions_radians = {0, 0, 0, 0};

    @Override public void updateInputs() {
        updateSim();
        inputs.calibrating = false;
        inputs.moving = !sim_odometry.getTranslation().equals(last_sim_odometry.getTranslation());
        inputs.rotating = !sim_odometry.getRotation().equals(last_sim_odometry.getRotation());

        inputs.timestamp = (long)(Timer.getFPGATimestamp() * 100);
        inputs.angle_adjustment = super.getAngleAdjustment();
        inputs.temperature_celcius = 0.0;
        inputs.angle = sim_odometry.getRotation();
        inputs.roll_radians = 0.0;
        inputs.pitch_radians = 0.0;
        inputs.yaw_radians = sim_odometry.getRotation().getRadians();

        inputs.velocity_x = sim_odometry.minus(last_sim_odometry).getX();
        inputs.velocity_y = sim_odometry.minus(last_sim_odometry).getY();
        inputs.velocity_z = 0.0;

        inputs.displacement_x = sim_odometry.getX();
        inputs.displacement_y = sim_odometry.getY();
        inputs.displacement_z = 0.0;
    }

    private void updateSim() {
        last_sim_odometry = sim_odometry;
        SubSystems.swerve_drive.getModules();

        Rotation2d[] turnPositions = new Rotation2d[4];
        for (int i = 0; i < 4; i++)
            turnPositions[i] = SubSystems.swerve_drive.getModules()[i].getPosition().angle;

        SwerveModuleState[] measuredStatesDiff = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            measuredStatesDiff[i] = new SwerveModuleState(
                    (SubSystems.swerve_drive.getModules()[i].getPosition().distanceMeters - last_module_positions_radians[i]),
                    turnPositions[i]);
            last_module_positions_radians[i] = SubSystems.swerve_drive.getModules()[i].getPosition().distanceMeters;
        }
        // sim_odometry = sim_odometry.exp(SubSystems.swerve_drive.getKinematics().toTwist2d(SubSystems.swerve_drive.getModulePositions()));
        ChassisSpeeds speeds = SubSystems.swerve_drive.getKinematics().toChassisSpeeds(measuredStatesDiff);
        Twist2d twist = new Twist2d(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond);
        sim_odometry = sim_odometry.exp(twist);
    }
    @Override public Rotation2d getRotation2d() { return inputs.angle.plus(Rotation2d.fromDegrees(inputs.angle_adjustment)); }
}
