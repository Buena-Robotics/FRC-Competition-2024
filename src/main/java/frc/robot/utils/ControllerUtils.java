package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class ControllerUtils {
    
    public static final CommandXboxController commandController = new CommandXboxController(Constants.Controller.XBOXCONTROLLERPORT);
    public static final XboxController controller = new XboxController(Constants.Controller.XBOXCONTROLLERPORT);
    
    public static final JoystickButton aButton = new JoystickButton(controller, Constants.Controller.A.value);
    public static final JoystickButton bButton = new JoystickButton(controller, Constants.Controller.B.value);
    public static final JoystickButton yButton = new JoystickButton(controller, Constants.Controller.Y.value);
    public static final JoystickButton xButton = new JoystickButton(controller, Constants.Controller.X.value);
    public static final JoystickButton rBButton = new JoystickButton(controller, Constants.Controller.RB.value);
    public static final JoystickButton lBButton = new JoystickButton(controller, Constants.Controller.LB.value);
    public static final JoystickButton lSButton = new JoystickButton(controller, Constants.Controller.LS.value);
    public static final JoystickButton navButton = new JoystickButton(controller, Constants.Controller.LS.value);
    public static final JoystickButton startButton = new JoystickButton(controller, Constants.Controller.MENU.value);
    public static final JoystickButton rSButton = new JoystickButton(controller, Constants.Controller.NAVIGATION.value);

    public static final JoystickButton rTTrigger = new JoystickButton(controller, Constants.Controller.RT.value);
    public static final JoystickButton lTTrigger = new JoystickButton(controller, Constants.Controller.RT.value);
    public static final JoystickButton lS_X = new JoystickButton(controller, Constants.Controller.LS_X.value);
    public static final JoystickButton lS_Y = new JoystickButton(controller, Constants.Controller.LS_Y.value);

    public static final Trigger dpadUpButton = new Trigger(() -> getDpadDirection().equals(DPadDirection.UP));
    public static final Trigger dpadDownButton = new Trigger(() -> getDpadDirection().equals(DPadDirection.DOWN));
    public static final Trigger dpadLeftButton = new Trigger(() -> getDpadDirection().equals(DPadDirection.LEFT));
    public static final Trigger dpadRightButton = new Trigger(() -> getDpadDirection().equals(DPadDirection.RIGHT));

    // Dpad directions are stored in degrees
    public static int getDpad() {
        return controller.getPOV();
    }

    public static DPadDirection getDpadDirection() {
        for (DPadDirection dir : DPadDirection.values()) {
            if (dir.get() == getDpad()) return dir;
        }

        return DPadDirection.NONE;
    }

    // Check if the current dpad pov equals the given direction's pov
    public static boolean isDpadPressed(DPadDirection dir) {
        return getDpad() == dir.get();
    }

    // Just helpful for readability
    public static enum DPadDirection {
        UP(0), DOWN(180), LEFT(270), RIGHT(90), NONE(-1);

        private int pov;

        public int get() {
            return pov;
        }

        private DPadDirection(int pov) {
            this.pov = pov;
        }
    }

    public static void setBindings() {
        
    }
}
