package frc.robot.subsystems;

import java.util.function.Consumer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class ClimbSubsystem extends SubsystemBase {
    
    private final int WINCH_MOTOR_ID = 12;
    private final int BORE_ENCODER_CHANNEL = 0;

    private final CANSparkMax winch_motor = new CANSparkMax(WINCH_MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder winch_encoder = winch_motor.getEncoder();
    private final DutyCycleEncoder bore_encoder = new DutyCycleEncoder(BORE_ENCODER_CHANNEL);

    public boolean armLocked = false;
    //1/2 inch hex shaft
    // private static final double WINCH_ENCODER_ROTATION_TO_METERS = 64/1 * Math.PI * Units.inchesToMeters(27.5) * 2;
    private static final double WINCH_ENCODER_ROTATION_TO_RADIANS = 1/64;

    public ClimbSubsystem() {
        winch_motor.setIdleMode(IdleMode.kBrake);
        winch_encoder.setPosition(0);
        winch_encoder.setPositionConversionFactor(1/64);
    }

    private void drive(Measure<Voltage> volts){}
    private void log(SysIdRoutineLog sysid_log){}

    private final Mechanism mechanism = new Mechanism(this::drive, this::log, this);
    private final SysIdRoutine.Config config = new SysIdRoutine.Config();
    private final SysIdRoutine routine = new SysIdRoutine(config, mechanism);

    public Command sysIdQuasistaticForward(){ return routine.quasistatic(Direction.kForward); }
    public Command sysIdQuasistaticReverse(){ return routine.quasistatic(Direction.kReverse); }
    public Command sysIdDynamicForward(){ return routine.dynamic(Direction.kForward); }
    public Command sysIdDynamicReverse(){ return routine.dynamic(Direction.kReverse); }

    @Override public void periodic() {
        SmartDashboard.putNumber("arm_encoder", getAngleDegrees());
        SmartDashboard.putNumber("winch_encoder", winch_encoder.getPosition());
    }

    public void moveArm(double speed, boolean lock, boolean shooting) {
        // unlockArm();
        // if(speed < 0 && Units.radiansToDegrees(winch_encoder.getPosition()) <= 5);
        winch_motor.set(speed);

        // if (lock) lockArm(shooting);
    }

    public void moveArmToPosition(ArmPosition position) {
        if (getAngleDegrees() < position.getEncoderPos()) {
            moveArm(0.5, true, true);
        }
    }

    public double getAngleDegrees() {
        return (bore_encoder.getAbsolutePosition()) * (90); 
    }


    public void lockArm(boolean shooting) {
        winch_motor.setIdleMode(IdleMode.kBrake);
    }

    public void unlockArm() {
        winch_motor.setIdleMode(IdleMode.kCoast);
    }

    enum ArmPosition {
        DOWN(0), 
        SPEAKER_CLOSE(0), 
        SPEAKER_STAGE(0), 
        UP(0);

        double encoderPos;

        private ArmPosition(double encoderPos) {
            this.encoderPos = encoderPos;
        }

        public double getEncoderPos() {
            return encoderPos;
        };
    }
}
