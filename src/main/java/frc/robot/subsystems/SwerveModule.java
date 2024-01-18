package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;

public class SwerveModule {
    public CANSparkMax speedMotor;
    public CANSparkMax angleMotor;
    public PIDController pidController;
    public final double MAX_VOLTS = 4.95;

    public SwerveModule(int angleMotorId, int speedMotorId, int encoder) {
        this.angleMotor = new CANSparkMax(angleMotorId, MotorType.kBrushless);
        this.speedMotor = new CANSparkMax(speedMotorId, MotorType.kBrushless);
        pidController = new PIDController(1, 0, 0);

        pidController.enableContinuousInput(-1, 1);
    }

    public void drive (double speed, double angle) {
        speedMotor.set (speed);
    
        double setpoint = angle * (MAX_VOLTS * 0.5) + (MAX_VOLTS * 0.5); // Optimization offset can be calculated here.
        if (setpoint < 0) {
            setpoint = MAX_VOLTS + setpoint;
        }
        if (setpoint > MAX_VOLTS) {
            setpoint = setpoint - MAX_VOLTS;
        }
    
        pidController.setSetpoint (setpoint);
    }
    
}
