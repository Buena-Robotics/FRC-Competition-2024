package frc.robot.subsystems.notearm;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.Constants.SubSystems;
import frc.robot.commands.XStop;
import frc.robot.subsystems.climber.Climb.ArmPosition;
import frc.robot.utils.TunableNumber;

public abstract class NoteArm extends SubsystemBase {
    @AutoLog public static class NoteArmInputs {
        public boolean is_claw_open = true;
        public boolean is_arm_out = false;
        public boolean is_arm_up = false;

        public boolean note_end_beam_broke = false;

        public double color_sensor_color[] = new double[3];
        public int color_sensor_red_raw = 0;
        public int color_sensor_green_raw = 0;
        public int color_sensor_blue_raw = 0;
        public int color_sensor_ir_raw = 0;
        public int color_sensor_proximity_raw = 2047;

        public double compressor_applied_volts = 0.0;
        public double compressor_current_amps = 0.0;
    }

    private static final TunableNumber note_hue_lower_threshold = new TunableNumber("NoteArm/NoteHueLowerThresh", 10);
    private static final TunableNumber note_hue_upper_threshold = new TunableNumber("NoteArm/NoteHueUpperThresh", 90);
    private static final TunableNumber note_saturation_threshold = new TunableNumber("NoteArm/NoteSatThresh", 100);
    private static final TunableNumber note_value_threshold = new TunableNumber("NoteArm/NoteValueThresh", 80);

    private final Mechanism2d color_sensor_mechanism = new Mechanism2d(10, 10, new Color8Bit()); 

    private static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;
    private static final int PNEUMATIC_MODULE_ID = 11;
    private static final int CLAW_SOLENOID_FORWARD_CHANNEL    = 0;
    private static final int CLAW_SOLENOID_REVERSE_CHANNEL    = 1;
    private static final int ARM_UP_SOLENOID_FORWARD_CHANNEL  = 2;
    private static final int ARM_UP_SOLENOID_REVERSE_CHANNEL  = 3;
    private static final int ARM_OUT_SOLENOID_FORWARD_CHANNEL = 4;
    private static final int ARM_OUT_SOLENOID_REVERSE_CHANNEL = 5;
    private static final I2C.Port COLOR_SENSOR_PORT = I2C.Port.kOnboard;
    private static final int NOTE_END_BREAKER_ID = 4;    

    protected final PneumaticHub hub = new PneumaticHub(PNEUMATIC_MODULE_ID);
    protected final Compressor compressor = new Compressor(PNEUMATIC_MODULE_ID, PNEUMATICS_MODULE_TYPE);
    protected final DoubleSolenoid claw_solenoid    = new DoubleSolenoid(PNEUMATIC_MODULE_ID, PNEUMATICS_MODULE_TYPE, CLAW_SOLENOID_FORWARD_CHANNEL, CLAW_SOLENOID_REVERSE_CHANNEL);
    protected final DoubleSolenoid arm_up_solenoid  = new DoubleSolenoid(PNEUMATIC_MODULE_ID, PNEUMATICS_MODULE_TYPE, ARM_UP_SOLENOID_FORWARD_CHANNEL, ARM_UP_SOLENOID_REVERSE_CHANNEL);
    protected final DoubleSolenoid arm_out_solenoid = new DoubleSolenoid(PNEUMATIC_MODULE_ID, PNEUMATICS_MODULE_TYPE, ARM_OUT_SOLENOID_FORWARD_CHANNEL, ARM_OUT_SOLENOID_REVERSE_CHANNEL);
    protected final ColorSensorV3 color_sensor      = new ColorSensorV3(COLOR_SENSOR_PORT);
    protected final DigitalInput note_end_beam_breaker = new DigitalInput(NOTE_END_BREAKER_ID);

    protected NoteArmInputsAutoLogged inputs = new NoteArmInputsAutoLogged();

    public NoteArm(){
        claw_solenoid.set(Value.kOff);
        arm_up_solenoid.set(Value.kOff);
        arm_out_solenoid.set(Value.kOff);

        color_sensor.configureProximitySensor(ProximitySensorResolution.kProxRes11bit, ProximitySensorMeasurementRate.kProxRate12ms);
        color_sensor.configureColorSensor(ColorSensorResolution.kColorSensorRes13bit, ColorSensorMeasurementRate.kColorRate25ms, GainFactor.kGain1x);

        SmartDashboard.putData("NoteArm/ColorSensor", color_sensor_mechanism);
        openClaw();
        new Thread(() -> this.autoClawThreadedLoop()).start();
    }

    protected void updateInputs(){
        inputs.is_claw_open = claw_solenoid.get() == Value.kForward;
        inputs.is_arm_out = arm_out_solenoid.get() == Value.kForward;
        inputs.is_arm_up = arm_up_solenoid.get() == Value.kForward;

        inputs.compressor_applied_volts = compressor.getAnalogVoltage();
        inputs.compressor_current_amps = compressor.getCurrent();
    }

    @Override public void periodic() {
        updateInputs();
        Logger.processInputs("NoteArm", inputs);

        color_sensor_mechanism.setBackgroundColor(new Color8Bit(getColor()));
        Logger.recordOutput("NoteArm/ColorSensorMechanism", color_sensor_mechanism);

    }

    private double proximityToMM(double x){
        if(x < 396) return 150.0;
        else if(x <= 444) return -2 * (x-473);
        else if(x <= 530) return -1/4.0 * (x - 680.0);
        return Math.pow(2, (-1/200.0) * (x - 1500)) + 8;
    }

    private boolean last_note_end_beam_broke = false;
    private double color_sensor_proximity_mm = 150.0;
    private void autoClawThreadedLoop(){
        while(true) {
        { // Update the necessary inputs
            inputs.note_end_beam_broke = !note_end_beam_breaker.get();
    
            inputs.color_sensor_color = new double[]{color_sensor.getColor().red, color_sensor.getColor().green, color_sensor.getColor().blue};
            inputs.color_sensor_red_raw = color_sensor.getRed();
            inputs.color_sensor_green_raw = color_sensor.getGreen();
            inputs.color_sensor_blue_raw = color_sensor.getBlue();
            inputs.color_sensor_ir_raw = color_sensor.getIR();
            inputs.color_sensor_proximity_raw = color_sensor.getProximity();
            color_sensor_proximity_mm = proximityToMM(color_sensor.getProximity());
        }

        if(last_note_end_beam_broke // Back Intake
                && !inputs.note_end_beam_broke 
                && isClawOpen() 
                && !RobotState.shooterHasNote()) 
            {
                if(!DriverStation.isAutonomousEnabled())
                    new XStop(SubSystems.swerve_drive, 680).schedule();
                closeClaw();
            }
        else if(color_sensor_proximity_mm < 30 // Front Intake
                && inputs.note_end_beam_broke 
                && isClawOpen() 
                && !RobotState.shooterHasNote())
            {
                if(!DriverStation.isAutonomousEnabled())
                    new XStop(SubSystems.swerve_drive, 680).schedule();
                closeClaw();
            }

        { // Update the previous inputs
            last_note_end_beam_broke = inputs.note_end_beam_broke;
        }
        Logger.recordOutput("NoteArm/Color Sensor Proximity MM", color_sensor_proximity_mm);
    }
    }

    public boolean hasNote(){ return !inputs.is_claw_open; }

    public boolean isClawOpen(){ return inputs.is_claw_open; }
    public boolean isArmUp()   { return inputs.is_arm_up; }
    public boolean isArmOut()  { return inputs.is_arm_out; }

    public void openClaw()  { claw_solenoid.set(Value.kForward); }
    public void moveArmUp() { arm_up_solenoid.set(Value.kForward); }
    public void moveArmOut(){ arm_out_solenoid.set(Value.kForward); }

    public void closeClaw()  { if(!RobotState.shooterHasNote()) claw_solenoid.set(Value.kReverse); }
    public void moveArmDown(){ arm_up_solenoid.set(Value.kReverse); }
    public void moveArmIn()  { arm_out_solenoid.set(Value.kReverse); }

    public Color getColor(){ return new Color(inputs.color_sensor_color[0], inputs.color_sensor_color[1], inputs.color_sensor_color[2]); }
    public int[] getColorSensorRGB(){
        final Color color = color_sensor.getColor();
        return new int[]{(int)(color.red*255), (int)(color.green*255), (int)(color.blue*255)};
    }


    public boolean detectingNoteColor(){
        int[] rgb = getColorSensorRGB();
        final float[] HSV = java.awt.Color.RGBtoHSB(rgb[0], rgb[1], rgb[2], null);

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

    public Command grabNoteCommand()  { return this.runOnce(() -> { closeClaw(); }); }
    public Command pushArmUpCommand() { return this.runOnce(() -> { moveArmUp(); }); }
    public Command pushArmOutCommand(){ return this.runOnce(() -> { moveArmOut(); }); }

    public Command releaseNoteCommand(){ return this.runOnce(() -> { openClaw(); }); }
    public Command pullArmInCommand()  { return this.runOnce(() -> { moveArmIn(); }); }
    public Command pullArmDownCommand(){ return this.runOnce(() -> { moveArmDown(); }); }

    public Command grabNoteFullCommand(){
        return grabNoteCommand()
            .andThen(
                new ParallelCommandGroup(
                    SubSystems.climb.moveArmToPosition(ArmPosition.UP),
                    new WaitCommand(0.65)
                ),
                pullArmInCommand(),
                new WaitCommand(0.10),
                pushArmUpCommand(),
                new WaitCommand(0.30),
                pushArmOutCommand()
            );
    }
    public Command releaseNoteFullCommand(){
        return releaseNoteCommand()
                .andThen(
                    new ParallelCommandGroup(
                        SubSystems.climb.moveArmToPosition(ArmPosition.UP), 
                        new WaitCommand(2.00)
                    ),
                    pullArmInCommand(),
                    new WaitCommand(0.50),
                    pullArmDownCommand(),
                    new WaitCommand(0.05),
                    pushArmOutCommand()
                );
    }
}
