package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private static final int FEED_MOTOR_ID = 10;
    private static final int LAUNCH_MOTOR_ID = 9;
    private static final int FEED_SMART_CURRENT_LIMIT = 80; 
    private static final int LAUNCH_SMART_CURRENT_LIMIT = 80;
    public static final double FEED_SPEED = 1;
    public static final double LAUNCH_SPEED = 1;
    public static final double INTAKE_LAUNCH_SPEED = -1; 
    public static final double INTAKE_FEED_SPEED = -.2;

    // private static final int BORE_ENCODER_CHANNEL = 0;

    public final CANSparkMax feed_motor = new CANSparkMax(FEED_MOTOR_ID, MotorType.kBrushless);;
    public final CANSparkMax launch_motor = new CANSparkMax(LAUNCH_MOTOR_ID, MotorType.kBrushless);
    public final RelativeEncoder feed_motor_encoder;
    public final RelativeEncoder launch_motor_encoder;

    // public final DutyCycleEncoder bore_encoder = new DutyCycleEncoder(BORE_ENCODER_CHANNEL);

    private double position_before_clearing_for_note_arm = 0;

    public ShooterSubsystem() {
        feed_motor_encoder = feed_motor.getEncoder();
        launch_motor_encoder = launch_motor.getEncoder();

        feed_motor.setSmartCurrentLimit(FEED_SMART_CURRENT_LIMIT);
        launch_motor.setSmartCurrentLimit(LAUNCH_SMART_CURRENT_LIMIT);

        // bore_encoder.setPositionOffset(-1);
        // bore_encoder.setDistancePerRotation(-1);
        // bore_encoder.setDistancePerRotation(-1);
        // bore_encoder.reset();
    }

    public double getFeedMotorVelocity(){ return feed_motor_encoder.getVelocity(); }
    public double getLaunchMotorVelocity(){ return feed_motor_encoder.getVelocity(); }

    public Command intakeCommand() {
        return this.startEnd(() -> {
            feed_motor.set(INTAKE_FEED_SPEED);
            launch_motor.set(INTAKE_LAUNCH_SPEED);
        }, () -> { stop(); });
    }

    public void stop() {
        feed_motor.set(0);
        launch_motor.set(0);
    }

    // public double getAngleDegrees(){ return bore_encoder.getAbsolutePosition(); }

    // public Command moveToTop(){
    //     position_before_clearing_for_note_arm = getAngleDegrees();
    //     return moveToPosition(1);
    // }
    public Command resetPosition(){
        return moveToPosition(position_before_clearing_for_note_arm);
    }
    public Command moveToPosition(double position){
        return new Command(){
            @Override public void execute(){}
            @Override public boolean isFinished() { return true; }
        };
    }

    @Override public void periodic(){
        // SmartDashboard.putBoolean("Shooter/Connected", bore_encoder.isConnected());
        // SmartDashboard.putNumber("Shooter/Absolute Position", getAngleDegrees());
        // SmartDashboard.putNumber("Shooter/Distance", bore_encoder.getDistance());
        // SmartDashboard.putNumber("Shooter/DPR", bore_encoder.getDistancePerRotation());
    }

    @Override public void initSendable(SendableBuilder builder) {

    }
}
