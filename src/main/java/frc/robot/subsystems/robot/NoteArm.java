package frc.robot.subsystems.robot;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoteArm extends SubsystemBase {
    private static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;
    private static final int CLAW_SOLENOID_FORWARD_CHANNEL    = -1;
    private static final int CLAW_SOLENOID_REVERSE_CHANNEL    = -1;
    private static final int ARM_UP_SOLENOID_FORWARD_CHANNEL  = -1;
    private static final int ARM_UP_SOLENOID_REVERSE_CHANNEL  = -1;
    private static final int ARM_OUT_SOLENOID_FORWARD_CHANNEL = -1;
    private static final int ARM_OUT_SOLENOID_REVERSE_CHANNEL = -1;
    private static final I2C.Port COLOR_SENSOR_PORT   = I2C.Port.kOnboard;

    private final DoubleSolenoid claw_solenoid    = new DoubleSolenoid(PNEUMATICS_MODULE_TYPE, CLAW_SOLENOID_FORWARD_CHANNEL, CLAW_SOLENOID_REVERSE_CHANNEL);
    private final DoubleSolenoid arm_up_solenoid  = new DoubleSolenoid(PNEUMATICS_MODULE_TYPE, ARM_UP_SOLENOID_FORWARD_CHANNEL, ARM_UP_SOLENOID_REVERSE_CHANNEL);
    private final DoubleSolenoid arm_out_solenoid = new DoubleSolenoid(PNEUMATICS_MODULE_TYPE, ARM_OUT_SOLENOID_FORWARD_CHANNEL, ARM_OUT_SOLENOID_REVERSE_CHANNEL);
    private final ColorSensorV3 color_sensor = new ColorSensorV3(COLOR_SENSOR_PORT);

    public NoteArm(){
        // color_sensor.configureColorSensor(
        //     ColorSensorResolution.kColorSensorRes13bit, 
        //     ColorSensorMeasurementRate.kColorRate25ms, 
        //     GainFactor.kGain1x);
        // color_sensor.configureProximitySensorLED(
        //     LEDPulseFrequency.kFreq100kHz, 
        //     LEDCurrent.kPulse10mA, 
        //     0);
    }

    public boolean isClawOpen(){ return claw_solenoid.get() == Value.kForward; }
    public boolean isArmUp()   { return arm_up_solenoid.get() == Value.kForward; }
    public boolean isArmOut()  { return arm_out_solenoid.get() == Value.kForward; }

    public void openClaw()  { claw_solenoid.set(Value.kForward); }
    public void moveArmUp() { arm_up_solenoid.set(Value.kForward); }
    public void moveArmOut(){ arm_out_solenoid.set(Value.kForward); }

    public void closeClaw()  { claw_solenoid.set(Value.kReverse); }
    public void moveArmDown(){ arm_up_solenoid.set(Value.kReverse); }
    public void moveArmIn()  { arm_out_solenoid.set(Value.kReverse); }


    public double milimetersFromObject(){
        final int PROXIMITY_SENSOR_MAX_VALUE = 2047;
        final int MAX_DETECTING_DISTANCE_MM = 100; // 10 CM in specs
        final double PROXIMITY_VALUE_PER_MM = (double)PROXIMITY_SENSOR_MAX_VALUE / MAX_DETECTING_DISTANCE_MM; 
        int proximity_11bit = color_sensor.getProximity();

        return proximity_11bit * PROXIMITY_VALUE_PER_MM;
    }
    public boolean detectingNoteColor(){
        return false;
    }

    public boolean noteDetected(){
        return milimetersFromObject() < 10 && detectingNoteColor();
    }


    @Override public void periodic() {
        if(!isClawOpen() && !isArmUp() && !isArmOut() && noteDetected()) closeClaw();
    }


    @Override public void initSendable(SendableBuilder builder) {

    }
}
