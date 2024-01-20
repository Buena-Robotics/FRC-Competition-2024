package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.devices.NavX;

import com.kauailabs.navx.frc.AHRS;

public class SwerveDriveSubsystem extends SubsystemBase {
    private static SwerveDriveSubsystem instance;
    public static SwerveDriveSubsystem getInstance(){ if(instance == null) instance = new SwerveDriveSubsystem(); return instance; }

    public static final int // DRIVE_MOTOR_IDS : ODDS
        FRONT_RIGHT_DRIVE_MOTOR_ID = 1,
        FRONT_LEFT_DRIVE_MOTOR_ID  = 3,
        BACK_RIGHT_DRIVE_MOTOR_ID  = 5,
        BACK_LEFT_DRIVE_MOTOR_ID   = 7;
    public static final int // TURN_MOTOR_IDS : EVENS
        FRONT_RIGHT_TURN_MOTOR_ID = 2,
        FRONT_LEFT_TURN_MOTOR_ID  = 4,
        BACK_RIGHT_TURN_MOTOR_ID  = 6,
        BACK_LEFT_TURN_MOTOR_ID   = 8;
    public static final boolean // DRIVE_MOTER_REVERSED
        FRONT_RIGHT_DRIVE_MOTER_REVERSED = false,
        FRONT_LEFT_DRIVE_MOTER_REVERSED  = true,
        BACK_RIGHT_DRIVE_MOTER_REVERSED  = false,
        BACK_LEFT_DRIVE_MOTER_REVERSED   = true;
    public static final boolean // TURN_MOTOR_REVERSED
        FRONT_RIGHT_TURN_MOTOR_REVERSED = true,
        FRONT_LEFT_TURN_MOTOR_REVERSED  = true,
        BACK_RIGHT_TURN_MOTOR_REVERSED  = true,
        BACK_LEFT_TURN_MOTOR_REVERSED   = true;
    public static final int // ABSOLUTE_ENCODER_IDS : 0-3
        FRONT_RIGHT_ABSOLUTE_ENCODER_ID = 0,
        FRONT_LEFT_ABSOLUTE_ENCODER_ID  = 1,
        BACK_RIGHT_ABSOLUTE_ENCODER_ID  = 2,
        BACK_LEFT_ABSOLUTE_ENCODER_ID   = 3;
    public static final double // ABSOLUTE_ENCODER_OFFSET_RADIANS
        FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS = 0.0,
        FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS  = 0.0,
        BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS  = 0.0,
        BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS   = 0.0;
    public static final boolean // ABSOLUTE_ENCODER_REVERSED
        FRONT_RIGHT_ABSOLUTE_ENCODER_REVERSED = false,
        FRONT_LEFT_ABSOLUTE_ENCODER_REVERSED  = false,
        BACK_RIGHT_ABSOLUTE_ENCODER_REVERSED  = false,
        BACK_LEFT_ABSOLUTE_ENCODER_REVERSED   = false;

    private final SwerveModule front_right = new SwerveModule (FRONT_RIGHT_DRIVE_MOTOR_ID, FRONT_RIGHT_TURN_MOTOR_ID, FRONT_RIGHT_DRIVE_MOTER_REVERSED, FRONT_RIGHT_TURN_MOTOR_REVERSED, 
                                                                FRONT_RIGHT_ABSOLUTE_ENCODER_ID, FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS, FRONT_RIGHT_ABSOLUTE_ENCODER_REVERSED );
    private final SwerveModule front_left  = new SwerveModule (FRONT_LEFT_DRIVE_MOTOR_ID, FRONT_LEFT_TURN_MOTOR_ID, FRONT_LEFT_DRIVE_MOTER_REVERSED, FRONT_LEFT_TURN_MOTOR_REVERSED, 
                                                                FRONT_LEFT_ABSOLUTE_ENCODER_ID, FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS, FRONT_LEFT_ABSOLUTE_ENCODER_REVERSED );
    private final SwerveModule back_right  = new SwerveModule (BACK_RIGHT_DRIVE_MOTOR_ID, BACK_RIGHT_TURN_MOTOR_ID, BACK_RIGHT_DRIVE_MOTER_REVERSED, BACK_RIGHT_TURN_MOTOR_REVERSED,
                                                                BACK_RIGHT_ABSOLUTE_ENCODER_ID, BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS, BACK_RIGHT_ABSOLUTE_ENCODER_REVERSED);
    private final SwerveModule back_left   = new SwerveModule (BACK_LEFT_DRIVE_MOTOR_ID, BACK_LEFT_TURN_MOTOR_ID, BACK_LEFT_DRIVE_MOTER_REVERSED, BACK_LEFT_TURN_MOTOR_REVERSED,
                                                                BACK_LEFT_ABSOLUTE_ENCODER_ID, BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS, BACK_LEFT_ABSOLUTE_ENCODER_REVERSED);

    // private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final NavX gyro = new NavX(SPI.Port.kMXP);


    public SwerveDriveSubsystem(){
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                long start_time = System.currentTimeMillis();
                while(gyro.isCalibrating()){ 
                    if(System.currentTimeMillis() - start_time > 20000)
                        System.err.println("Nav-X Unable to Calibrate in alloted time of 20 seconds");
                    break; 
                }
                zeroHeading();
            } catch (Exception e) { }
        }).start();
    }

    public void zeroHeading(){ gyro.reset(); }
    public double getHeading(){ return Math.IEEEremainder(gyro.getAngle(), 360); }

    public Rotation2d getRotation2d(){ return Rotation2d.fromDegrees(getHeading()); }

    @Override public void periodic() { 
        SmartDashboard.putNumber("Robot Heading", getHeading()); 
        SmartDashboard.putData(gyro);

    }
    @Override public void simulationPeriodic(){}

    @Override public void initSendable(SendableBuilder builder) {

    }

    public void stopModules(){
        front_left.stop();
        front_right.stop();
        back_left.stop();
        back_right.stop();
    }

    public void setModuleStates(SwerveModuleState[] desired_states){
        assert desired_states.length == 4 : "desired_states has invalid length";
        SwerveDriveKinematics.desaturateWheelSpeeds(desired_states, Constants.Drive.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
        front_left.setDesiredState (desired_states[0]);
        front_right.setDesiredState(desired_states[1]);
        back_left.setDesiredState  (desired_states[2]);
        back_right.setDesiredState (desired_states[3]);

        SmartDashboard.putString("front_left:", desired_states[0].toString());
        SmartDashboard.putString("front_right:", desired_states[1].toString());
        SmartDashboard.putString("back_left:", desired_states[2].toString());
        SmartDashboard.putString("back_right:", desired_states[3].toString());

    }
}
