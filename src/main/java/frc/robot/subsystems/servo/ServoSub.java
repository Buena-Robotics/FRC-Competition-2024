package frc.robot.subsystems.servo;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.TimerUtil;

public class ServoSub extends SubsystemBase {
    
    private final int SERVO_CHANNEL = 0;

    public Servo servo = new Servo(SERVO_CHANNEL);

    private TimerUtil timer = new TimerUtil();
    private int currentAngle = 0;

    public ServoSub() {
        
    }

    @Override
    public void periodic() {
        if (timer.hasTimeElapsed(1000, true)) {
            servo.setAngle(currentAngle);
            currentAngle = currentAngle == 180 ? 0 : 180;
        }
        super.periodic();
    }
}
