package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import java.awt.Color;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class NoteArmSubsystem extends SubsystemBase {
    private static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;
    private static final int CLAW_SOLENOID_FORWARD_CHANNEL    = 0;
    private static final int CLAW_SOLENOID_REVERSE_CHANNEL    = 1;
    private static final int ARM_UP_SOLENOID_FORWARD_CHANNEL  = 2;
    private static final int ARM_UP_SOLENOID_REVERSE_CHANNEL  = 3;
    private static final int ARM_OUT_SOLENOID_FORWARD_CHANNEL = 4;
    private static final int ARM_OUT_SOLENOID_REVERSE_CHANNEL = 5;
    private static final I2C.Port COLOR_SENSOR_PORT   = I2C.Port.kOnboard;
    private static final int NOTE_DISTANCE_THRESHOLD_MM = 10;
    private static final int NOTE_HUE_LOWER_THRESHOLD   = 50;
    private static final int NOTE_HUE_UPPER_THRESHOLD   = 345;
    private static final int NOTE_SATURATION_THRESHOLD  = 100;
    private static final int NOTE_VALUE_THRESHOLD       = 80;
    private static final double DELAY = 0.5;

    private final DoubleSolenoid claw_solenoid    = new DoubleSolenoid(PNEUMATICS_MODULE_TYPE, CLAW_SOLENOID_FORWARD_CHANNEL, CLAW_SOLENOID_REVERSE_CHANNEL);
    private final DoubleSolenoid arm_up_solenoid  = new DoubleSolenoid(PNEUMATICS_MODULE_TYPE, ARM_UP_SOLENOID_FORWARD_CHANNEL, ARM_UP_SOLENOID_REVERSE_CHANNEL);
    private final DoubleSolenoid arm_out_solenoid = new DoubleSolenoid(PNEUMATICS_MODULE_TYPE, ARM_OUT_SOLENOID_FORWARD_CHANNEL, ARM_OUT_SOLENOID_REVERSE_CHANNEL);
    private final ColorSensorV3 color_sensor      = new ColorSensorV3(COLOR_SENSOR_PORT);

    public NoteArmSubsystem(){
        claw_solenoid.set(Value.kOff);
        arm_up_solenoid.set(Value.kOff);
        arm_out_solenoid.set(Value.kOff);
    }

    public boolean isClawOpen(){ return claw_solenoid.get() == Value.kForward; }
    public boolean isArmUp()   { return arm_up_solenoid.get() == Value.kForward; }
    public boolean isArmOut()  { return arm_out_solenoid.get() == Value.kForward; }

    public void openClaw()  { claw_solenoid.set(Value.kReverse); }
    public void moveArmUp() { arm_up_solenoid.set(Value.kForward); }
    public void moveArmOut(){ arm_out_solenoid.set(Value.kForward); }

    public void closeClaw()  { claw_solenoid.set(Value.kForward); }
    public void moveArmDown(){ arm_up_solenoid.set(Value.kReverse); }
    public void moveArmIn()  { arm_out_solenoid.set(Value.kReverse); }


    public double milimetersFromObject(){
        final int PROXIMITY_SENSOR_MAX_VALUE = 2047;
        final int MAX_DETECTING_DISTANCE_MM = 100; // 10 CM in specs
        final double PROXIMITY_VALUE_PER_MM = (double)PROXIMITY_SENSOR_MAX_VALUE / MAX_DETECTING_DISTANCE_MM; 
        final int proximity_11bit = color_sensor.getProximity();

        return proximity_11bit * PROXIMITY_VALUE_PER_MM;
    }

    static final double _20BIT_TO_8BIT_SCALE_FACTOR = Math.pow(2.0, 8.0) / Math.pow(2.0, 20.0); 
    private int _20bitTo8bit(int value20bit){ return (int)(value20bit * _20BIT_TO_8BIT_SCALE_FACTOR); }
    private int getColorSensorRed8bit()  { return _20bitTo8bit(color_sensor.getRed()); }
    private int getColorSensorGreen8bit(){ return _20bitTo8bit(color_sensor.getGreen()); }
    private int getColorSensorBlue8bit() { return _20bitTo8bit(color_sensor.getBlue()); }

    public boolean detectingNoteColor(){
        final float[] HSV = Color.RGBtoHSB(getColorSensorRed8bit(), getColorSensorGreen8bit(), getColorSensorBlue8bit(), null);
        if(HSV[0] < NOTE_HUE_LOWER_THRESHOLD || HSV[0] > NOTE_HUE_UPPER_THRESHOLD)
            if(HSV[1] > NOTE_SATURATION_THRESHOLD && HSV[2] > NOTE_VALUE_THRESHOLD) return true;
        return false;
    }

    public boolean noteDetected(){
        return milimetersFromObject() < NOTE_DISTANCE_THRESHOLD_MM && detectingNoteColor();
    }


    @Override public void periodic() {
        if(!isClawOpen() && !isArmUp() && isArmOut() && noteDetected()) closeClaw();
    }

    public Command grabNoteCommand()  { return this.runOnce(() -> { closeClaw(); }); }
    public Command pushArmUpCommand() { return this.runOnce(() -> { moveArmUp(); }); }
    public Command pushArmOutCommand(){ return this.runOnce(() -> { moveArmOut(); }); }

    public Command releaseNoteCommand(){ return this.runOnce(() -> { openClaw(); }); }
    public Command pullArmInCommand()  { return this.runOnce(() -> { moveArmIn(); }); }
    public Command pullArmDownCommand(){ return this.runOnce(() -> { moveArmDown(); }); }

    public Command grabNoteFullCommand(){
        if(isArmUp() || !isArmOut()) return this.runOnce(() -> {});
        return grabNoteCommand()
            .andThen(pullArmInCommand())
            .andThen(new WaitCommand(DELAY))
            .andThen(pushArmUpCommand())
            .andThen(new WaitCommand(DELAY))
            .andThen(pushArmOutCommand());
    }

    public Command releaseNoteFullCommand(){
        if(isClawOpen() || !isArmUp() || !isArmOut()) return this.runOnce(() -> {});
        return releaseNoteCommand()
            .andThen(new WaitCommand(DELAY))
            .andThen(pullArmInCommand())
            .andThen(new WaitCommand(DELAY))
            .andThen(pullArmDownCommand())
            .andThen(new WaitCommand(DELAY))
            .andThen(pushArmOutCommand());
        }

    @Override public void initSendable(SendableBuilder builder) {

    }
}
