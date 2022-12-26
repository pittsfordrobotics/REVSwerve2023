package frc.robot.util.controller;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.HashMap;

public class BetterXboxController extends XboxController {
    public final JoystickButton A;
    public final JoystickButton B;
    public final JoystickButton X;
    public final JoystickButton Y;
    public final JoystickButton LB;
    public final JoystickButton RB;
    public final BetterPOVButton DUp;
    public final BetterPOVButton DRight;
    public final BetterPOVButton DDown;
    public final BetterPOVButton DLeft;
    public final TriggerButton LT;
    public final TriggerButton RT;
    public final JoystickButton Start;
    public final JoystickButton Back;

    private static final HashMap<Humans, BetterXboxController> controllers = new HashMap<>();

    public enum Hand {
        RIGHT, LEFT
    }

    public enum Humans {
        DRIVER, OPERATOR
    }

    public BetterXboxController(int port, Humans humans) {
        super(port);
        A = new JoystickButton(this, XboxController.Button.kA.value);
        B = new JoystickButton(this, XboxController.Button.kB.value);
        X = new JoystickButton(this, XboxController.Button.kX.value);
        Y = new JoystickButton(this, XboxController.Button.kY.value);
        LB = new JoystickButton(this, XboxController.Button.kLeftBumper.value);
        RB = new JoystickButton(this, XboxController.Button.kRightBumper.value);
        DUp = new BetterPOVButton(this, 0);
        DRight = new BetterPOVButton(this, 90);
        DDown = new BetterPOVButton(this, 180);
        DLeft = new BetterPOVButton(this, 270);
        LT = new TriggerButton(this, Hand.LEFT);
        RT = new TriggerButton(this, Hand.RIGHT);
        Back = new JoystickButton(this, XboxController.Button.kBack.value);
        Start = new JoystickButton(this, XboxController.Button.kStart.value);
        controllers.put(humans, this);
    }

    public static BetterXboxController getController(Humans humans) {
        return controllers.get(humans);
    }

    /** @param value between 0 and 1 */
    public void setRumble(double value) {
        setRumble(RumbleType.kLeftRumble, value);
        setRumble(RumbleType.kRightRumble, value);
    }
}