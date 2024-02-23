package frc.robot.subsystems;

import java.util.function.Consumer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
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

    public ClimbSubsystem() {
        winch_motor.setIdleMode(IdleMode.kBrake);
        // winch_encoder.setPosition(getBoreAngleDegrees());
        winch_encoder.setPositionConversionFactor(3.0 / 320.0);
    }

    @Override public void periodic() {
        SmartDashboard.putNumber("arm_encoder", getBoreAngleDegrees());
        SmartDashboard.putNumber("winch_encoder", getWinchAngleDegrees());
    }

    public void moveArm(double speed, boolean lock, boolean shooting) {
        // unlockArm();
        // if(speed < 0 && Units.radiansToDegrees(winch_encoder.getPosition()) <= 5);
        winch_motor.set(speed);

        // if (lock) lockArm(shooting);
    }

    public double getSetpointVoltage(double setpoint){
        double measurement = getBoreAngleDegrees();
        if(measurement - setpoint < 0)
            return Math.sqrt(-measurement + setpoint);
        else
            return -Math.sqrt(measurement - setpoint);
    }
    public void runSetpoint(double setpoint){
        double voltage = MathUtil.clamp(getSetpointVoltage(setpoint), -12, 12);
        // double voltage = MathUtil.clamp(getSetpointVoltage(setpoint), -12, 12);
        winch_motor.setVoltage(voltage);
    }

    public Command moveArmToPosition(ArmPosition position) {
        return new Command() {
            private final double setpoint = position.getPosDegrees();
            private double measurement = getBoreAngleDegrees();

            @Override public void execute() {
                runSetpoint(setpoint);
                this.measurement = getBoreAngleDegrees();
            }
            @Override public void end(boolean interrupted) {
                winch_motor.setVoltage(0);
            }
            @Override public boolean isFinished() {
                return Math.abs(measurement - setpoint) < 3;
            }
        };
        // double setpoint = position.getPosDegrees();
        // runSetpoint(setpoint);
        // double measurement = getBoreAngleDegrees();
        // if(Math.abs(measurement - setpoint) < 5)
        //     winch_motor.setVoltage(0);
    }

    public double getBoreAngleDegrees() {
        return 369 * (bore_encoder.getAbsolutePosition() - 0.146338);
    }
    public double getWinchAngleDegrees(){
        return winch_encoder.getPosition();
    }


    public void lockArm(boolean shooting) {
        winch_motor.setIdleMode(IdleMode.kBrake);
    }

    public void unlockArm() {
        winch_motor.setIdleMode(IdleMode.kCoast);
    }

    public enum ArmPosition {
        DOWN(64.127554), 
        SPEAKER_CLOSE(38.472514),
        SPEAKER_STAGE(49.373346),
        UP(0);

        double degrees;

        private ArmPosition(double degrees) {
            this.degrees = degrees;
        }

        public double getPosDegrees() {
            return degrees;
        };
    }
}
