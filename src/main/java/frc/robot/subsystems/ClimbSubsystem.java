package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    
    private final int WINCH_MOTOR_ID = 12;
    private final int BORE_ENCODER_CHANNEL = 0;

    public final CANSparkMax winchMotor = new CANSparkMax(WINCH_MOTOR_ID, MotorType.kBrushless);
    public final DutyCycleEncoder boreEncoder = new DutyCycleEncoder(BORE_ENCODER_CHANNEL);


    public boolean armLocked = false;

    public ClimbSubsystem() {

    }

    public void moveArm(double speed, boolean lock, boolean shooting) {
        // unlockArm();

        winchMotor.set(speed);

        if (lock) lockArm(shooting);
    }

    public void moveArmToPosition(ArmPosition position) {
        if (getAngleDegrees() < position.getEncoderPos()) {
            moveArm(0.5, true, true);
        }
    }

    public double getAngleDegrees() {
        return boreEncoder.getAbsolutePosition(); 
    }


    public void lockArm(boolean shooting) {
        winchMotor.setIdleMode(IdleMode.kBrake);
    }

    public void unlockArm() {
        winchMotor.setIdleMode(IdleMode.kCoast);
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
