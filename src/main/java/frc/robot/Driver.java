package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Driver extends XboxController {
    public Driver() {
        super(0);
    }
    public Trigger Putter(){
        return new Trigger(this::getXButton);
    }
    public Trigger PutterCorrection(){
        return new Trigger(this::getLeftBumperButton);
    }
}