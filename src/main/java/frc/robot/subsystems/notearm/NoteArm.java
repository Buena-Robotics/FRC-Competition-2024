package frc.robot.subsystems.notearm;


import java.awt.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.utils.TunableNumber;

public class NoteArm extends SubsystemBase {
    private static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;
    private static final int PNEUMATIC_MODULE_ID = 11;
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
    private static final double DELAY = 1;

    private final PneumaticHub hub = new PneumaticHub(PNEUMATIC_MODULE_ID);
    private final Compressor compressor = new Compressor(PNEUMATIC_MODULE_ID, PNEUMATICS_MODULE_TYPE);
    private final DoubleSolenoid claw_solenoid    = new DoubleSolenoid(PNEUMATIC_MODULE_ID, PNEUMATICS_MODULE_TYPE, CLAW_SOLENOID_FORWARD_CHANNEL, CLAW_SOLENOID_REVERSE_CHANNEL);
    private final DoubleSolenoid arm_up_solenoid  = new DoubleSolenoid(PNEUMATIC_MODULE_ID, PNEUMATICS_MODULE_TYPE, ARM_UP_SOLENOID_FORWARD_CHANNEL, ARM_UP_SOLENOID_REVERSE_CHANNEL);
    private final DoubleSolenoid arm_out_solenoid = new DoubleSolenoid(PNEUMATIC_MODULE_ID, PNEUMATICS_MODULE_TYPE, ARM_OUT_SOLENOID_FORWARD_CHANNEL, ARM_OUT_SOLENOID_REVERSE_CHANNEL);
    private final ColorSensorV3 color_sensor      = new ColorSensorV3(COLOR_SENSOR_PORT);
    
    public NoteArm(){
        // color_sensor.configureColorSensor(ColorSensorResolution.k, null, null);
        // color_sensor.configureProximitySensor(ProximitySensorResolution.kProxRes11bit, ProximitySensorMeasurementRate.);
        claw_solenoid.set(Value.kOff);
        arm_up_solenoid.set(Value.kOff);
        arm_out_solenoid.set(Value.kOff);
        hub.enableCompressorDigital();
        compressor.enableDigital();
    }

    public boolean isClawOpen(){ return claw_solenoid.get() == Value.kReverse; }
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
        final double PROXIMITY_VALUE_PER_MM = MAX_DETECTING_DISTANCE_MM / (double)PROXIMITY_SENSOR_MAX_VALUE; 
        final int proximity_11bit = 2047 - color_sensor.getProximity();

        SmartDashboard.putNumber("NoteArm/MMFromObject", (double)proximity_11bit);
        SmartDashboard.putNumber("NoteArm/CMFromObject", (double)proximity_11bit / 600);
        return 100 - (proximity_11bit * PROXIMITY_VALUE_PER_MM);
    }

    static final double _20BIT_TO_8BIT_SCALE_FACTOR = 4096; 
    private int _20bitTo8bit(int value20bit){ return (int)(value20bit / _20BIT_TO_8BIT_SCALE_FACTOR); }
    private int getColorSensorRed8bit()  { return _20bitTo8bit(color_sensor.getRed()); }
    private int getColorSensorGreen8bit(){ return _20bitTo8bit(color_sensor.getGreen()); }
    private int getColorSensorBlue8bit() { return _20bitTo8bit(color_sensor.getBlue()); }

    public boolean detectingNoteColor(){
        final float[] HSV = Color.RGBtoHSB(getColorSensorRed8bit(), getColorSensorGreen8bit(), getColorSensorBlue8bit(), null);
        SmartDashboard.putNumberArray("NoteArm/ColorDetectingHSV", new Double[]{(double)getColorSensorRed8bit(), (double)getColorSensorGreen8bit(), (double)getColorSensorBlue8bit()});
        if(HSV[0] < NOTE_HUE_LOWER_THRESHOLD || HSV[0] > NOTE_HUE_UPPER_THRESHOLD)
            if(HSV[1] > NOTE_SATURATION_THRESHOLD && HSV[2] > NOTE_VALUE_THRESHOLD) return true;
        return false;
    }

    public boolean noteDetected(){
        detectingNoteColor();
        return milimetersFromObject() < NOTE_DISTANCE_THRESHOLD_MM && detectingNoteColor();
    }


    @Override public void periodic() {
        SmartDashboard.putBoolean("NoteArm/NoteDetected", noteDetected());
        SmartDashboard.putBoolean("NoteArm/ColorSensorConnected", color_sensor.isConnected());

        // if(color_sensor.isConnected())
            // if(isClawOpen() && !isArmUp() && isArmOut() && noteDetected()) closeClaw();
        if(noteDetected()) closeClaw();
    }

    public Command grabNoteCommand()  { return this.runOnce(() -> { closeClaw(); }); }
    public Command pushArmUpCommand() { return this.runOnce(() -> { moveArmUp(); }); }
    public Command pushArmOutCommand(){ return this.runOnce(() -> { moveArmOut(); }); }

    public Command releaseNoteCommand(){ return this.runOnce(() -> { openClaw(); }); }
    public Command pullArmInCommand()  { return this.runOnce(() -> { moveArmIn(); }); }
    public Command pullArmDownCommand(){ return this.runOnce(() -> { moveArmDown(); }); }

    public Command grabNoteFullCommand(){
        // if(isArmUp() || !isArmOut()) return this.runOnce(() -> {});
        return grabNoteCommand()
            .andThen(new WaitCommand(DELAY))
            .andThen(pullArmInCommand())
            .andThen(new WaitCommand(DELAY))
            .andThen(pushArmUpCommand())
            .andThen(new WaitCommand(DELAY))
            .andThen(pushArmOutCommand());
    }
    private TunableNumber delay = new TunableNumber("NoteArmDelay", 0);
    public Command releaseNoteFullCommand(){
        // if(isClawOpen() || !isArmUp() || !isArmOut()) return this.runOnce(() -> {});
        return releaseNoteCommand()
            .andThen(new WaitCommand(DELAY))
            .andThen(pullArmInCommand())
            .andThen(new WaitCommand(DELAY))
            .andThen(pullArmDownCommand())
            .andThen(new WaitCommand(0.05))
            .andThen(pushArmOutCommand());
    }
}
