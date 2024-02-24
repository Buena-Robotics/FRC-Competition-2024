package frc.robot.subsystems.notearm;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.climber.Climb.ArmPosition;
import frc.robot.utils.TunableNumber;

public class NoteArm extends SubsystemBase {
    private static final TunableNumber note_distance_threshold_mm = new TunableNumber("NoteArm/NoteDistanceThreshMM", 2000);
    private static final TunableNumber note_hue_lower_threshold = new TunableNumber("NoteArm/NoteHueLowerThresh", 10);
    private static final TunableNumber note_hue_upper_threshold = new TunableNumber("NoteArm/NoteHueUpperThresh", 90);
    private static final TunableNumber note_saturation_threshold = new TunableNumber("NoteArm/NoteSatThresh", 100);
    private static final TunableNumber note_value_threshold = new TunableNumber("NoteArm/NoteValueThresh", 80);

    private static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;
    private static final int PNEUMATIC_MODULE_ID = 11;
    private static final int CLAW_SOLENOID_FORWARD_CHANNEL    = 0;
    private static final int CLAW_SOLENOID_REVERSE_CHANNEL    = 1;
    private static final int ARM_UP_SOLENOID_FORWARD_CHANNEL  = 2;
    private static final int ARM_UP_SOLENOID_REVERSE_CHANNEL  = 3;
    private static final int ARM_OUT_SOLENOID_FORWARD_CHANNEL = 4;
    private static final int ARM_OUT_SOLENOID_REVERSE_CHANNEL = 5;
    private static final I2C.Port COLOR_SENSOR_PORT   = I2C.Port.kOnboard;
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

    public boolean detectingNoteColor(){
        final Color color = color_sensor.getColor();
        final float[] HSV = java.awt.Color.RGBtoHSB((int)(color.red*255), (int)(color.green*255), (int)(color.blue*255),  null);

        SmartDashboard.putNumberArray("NoteArm/ColorRGBNormal", new Double[]{color.red, color.green, color.blue});
        SmartDashboard.putNumberArray("NoteArm/ColorRGB", new Double[]{color.red * 255, color.green * 255, color.blue * 255});
        SmartDashboard.putNumberArray("NoteArm/ColorHSV", new Double[]{(double)HSV[0], (double)HSV[1], (double)HSV[2]});
        SmartDashboard.putNumberArray("NoteArm/RangeHSV", new Double[]{
            note_hue_lower_threshold.get()/360, 
            note_hue_upper_threshold.get()/360, 
            note_saturation_threshold.get()/180, 
            note_value_threshold.get()/180
        });

        if(HSV[0] > note_hue_lower_threshold.get()/360 || HSV[0] < note_hue_upper_threshold.get()/360)
            if(HSV[1] > note_saturation_threshold.get()/180 && HSV[2] > note_value_threshold.get()/180) return true;
        return false;
    }

    public boolean noteDetected(){
        detectingNoteColor();
        return milimetersFromObject() < note_distance_threshold_mm.get() && detectingNoteColor();
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
        return 
        // new ParallelCommandGroup(
                // Constants.SubSystems.climb.moveArmToPosition(ArmPosition.UP),
                grabNoteCommand()
                .andThen(new WaitCommand(DELAY).andThen(pullArmInCommand()))
                // )
            .andThen(new WaitCommand(DELAY))
            .andThen(pushArmUpCommand())
            .andThen(new WaitCommand(DELAY))
            .andThen(pushArmOutCommand());
    }
    public Command releaseNoteFullCommand(){
        // if(isClawOpen() || !isArmUp() || !isArmOut()) return this.runOnce(() -> {});
        return releaseNoteCommand()
            .andThen(new WaitCommand(DELAY + 1.0))
            .andThen(pullArmInCommand())
            .andThen(new WaitCommand(DELAY)) 
            .andThen(pullArmDownCommand())
            .andThen(new WaitCommand(0.05))
            .andThen(pushArmOutCommand());
    }
}
