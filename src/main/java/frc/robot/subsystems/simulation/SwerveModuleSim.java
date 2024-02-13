package frc.robot.subsystems.simulation;

import frc.robot.Constants.Drive;
import frc.robot.subsystems.SwerveModuleIO;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class SwerveModuleSim extends SwerveModuleIO {
    private final CANSparkMax drive_motor;
    private final CANSparkMax turn_motor;

    private final RelativeEncoder drive_encoder;
    private final RelativeEncoder turn_encoder;

    public SwerveModuleSim(int drive_motor_id, int turn_motor_id, int absolute_encoder_id, double absolute_encoder_offset_radians) {
        super(absolute_encoder_id, absolute_encoder_offset_radians);

        this.drive_motor = new CANSparkMax(drive_motor_id, MotorType.kBrushless);
        this.turn_motor = new CANSparkMax(turn_motor_id, MotorType.kBrushless);

        this.drive_encoder = drive_motor.getEncoder();
        this.drive_encoder.setPositionConversionFactor(DRIVE_ENCODER_ROTATION_TO_METERS);
        this.drive_encoder.setVelocityConversionFactor(DRIVE_ENCODER_RPM_TO_METERS_PER_SECOND);
        
        this.turn_encoder = turn_motor.getEncoder();
        this.turn_encoder.setPositionConversionFactor(TURN_ENCODER_ROTATION_TO_RADIANS);
        this.turn_encoder.setVelocityConversionFactor(TURN_ENCODER_RPM_TO_RADIANS_PER_SECOND);

        resetEncoders();

        REVPhysicsSim.getInstance().addSparkMax(drive_motor, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(turn_motor, DCMotor.getNEO(1));
    }

    @Override public double getDrivePosition(){ return drive_encoder.getPosition(); }
    @Override public double getTurnPosition() { return turn_encoder.getPosition(); }

    @Override public double getDriveVelocity() { return drive_encoder.getVelocity(); }
    @Override public double getTurnVelocity()  { return  turn_encoder.getVelocity(); }

    @Override public void resetPosition(){drive_encoder.setPosition(0);}
    @Override public void resetEncoders(){
        drive_encoder.setPosition(0);
        turn_encoder.setPosition(getAbsoluteEncoderRadians());
    }

    @Override public void setDesiredState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond) < SET_STATE_SPEED_METERS_PER_SECOND_DEADBAND) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize( state, getState().angle );
        drive_motor.set( state.speedMetersPerSecond / Drive.PHYSICAL_MAX_SPEED_METERS_PER_SECOND );
        turn_motor.set( turn_controller.calculate( getTurnPosition(), state.angle.getRadians() ) );
    }

    @Override public void stop(){
        drive_motor.set(0);
        turn_motor.set(0);
    }
}
