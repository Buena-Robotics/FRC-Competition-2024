package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubSystems;

public class Climb extends SubsystemBase {
    protected static final double WINCH_ENCODER_GEAR_RATIO = 1 / 64.0;
    //Units.inchesToMeters(2.75)
    protected static final double WINCH_ENCODER_ROTATION_TO_RADIANS       = WINCH_ENCODER_GEAR_RATIO * Math.PI; 
    protected static final double WINCH_ENCODER_RPM_TO_RADIANS_PER_SECOND = WINCH_ENCODER_ROTATION_TO_RADIANS / 60; 
    private final int WINCH_MOTOR_ID = 12;
    private final int BORE_ENCODER_CHANNEL = 0;

    private final CANSparkMax winch_motor = new CANSparkMax(WINCH_MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder winch_encoder = winch_motor.getEncoder();
    private final DutyCycleEncoder bore_encoder = new DutyCycleEncoder(BORE_ENCODER_CHANNEL);

    
    public boolean armLocked = false;

    public Climb() {
        winch_motor.setIdleMode(IdleMode.kBrake);
        // winch_encoder.setPosition(getBoreAngleDegrees());
        winch_encoder.setPositionConversionFactor(1);
        winch_encoder.setVelocityConversionFactor(1);
        winch_encoder.setPosition(0);
    }

    @Override public void periodic() {
        SmartDashboard.putNumber("bore angle", getBoreAngleDegrees());
        SmartDashboard.putNumber("bore distance", bore_encoder.getDistance());
        SmartDashboard.putNumber("winch angle deg", getWinchAngleDegrees());
        SmartDashboard.putNumber("winch angle rad", getWinchAngleRadians());
        SmartDashboard.putNumber("winch velocity", winch_encoder.getVelocity());
        SmartDashboard.putNumber("winch voltage", winch_motor.getAppliedOutput());
    }

    public void moveArm(double speed, boolean lock, boolean shooting) {
        if(winch_encoder.getPosition() < 4 && speed < 0){
            winch_motor.set(0);
            DriverStation.reportWarning("Winch exceeded lower bounds: " + winch_encoder.getPosition(), false);
            return;
        }
        else if(winch_encoder.getPosition() > 112 && speed > 0){
            winch_motor.set(0);
            DriverStation.reportWarning("Winch exceeded lower bounds: " + winch_encoder.getPosition(), false);
            return;
        }
        winch_motor.set(speed);
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

        if(winch_encoder.getPosition() < 4 && voltage < 0){
            winch_motor.set(0);
            DriverStation.reportWarning("Winch exceeded lower bounds: " + winch_encoder.getPosition(), false);
            return;
        }
        else if(winch_encoder.getPosition() > 112 && voltage > 0){
            winch_motor.set(0);
            DriverStation.reportWarning("Winch exceeded lower bounds: " + winch_encoder.getPosition(), false);
            return;
        }
        // double voltage = MathUtil.clamp(getSetpointVoltage(setpoint), -12, 12);
        winch_motor.setVoltage(voltage);
    }

    public Command moveArmToPosition(ArmPosition position) {
        return new Command() {
            private final double setpoint = position.getPosDegrees();
            private double measurement = getBoreAngleDegrees();

            @Override
            public void initialize() {
                addRequirements(SubSystems.climb);
            }

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
    public double getBoreAngleRadians(){
        return Units.degreesToRadians(getBoreAngleDegrees());
    }
    public double getWinchAngleRadians(){
        return winch_encoder.getPosition();
    }
    public double getWinchAngleDegrees(){
        return Units.radiansToDegrees(getWinchAngleRadians());
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
