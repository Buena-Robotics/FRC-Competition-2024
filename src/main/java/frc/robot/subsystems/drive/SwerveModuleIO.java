package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public double drive_position_radians = 0.0;
        public double drive_velocity_radians_per_second = 0.0;
        public double drive_applied_volts = 0.0;
        public double[] drive_current_amps = new double[] {};
        public double[] drive_temp_celcius = new double[] {};

        public double turn_absolute_position_radians = 0.0;
        public double turn_position_radians = 0.0;
        public double turn_velocity_radians_per_second = 0.0;
        public double turn_applied_volts = 0.0;
        public double[] turn_current_amps = new double[] {};
        public double[] turn_temp_celcius = new double[] {};
    }

    /** Updates the set of loggable inputs. */
    public void updateInputs(ModuleIOInputs inputs);

    public default void setDriveVoltage(double volts){}
    public default void setTurnVoltage(double volts){}

    public default void setDriveBrakeMode(boolean enable){}
    public default void setTurnBrakeMode(boolean enable){}
}
