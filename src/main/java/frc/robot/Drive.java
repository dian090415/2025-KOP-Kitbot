package frc.robot;

import com.fasterxml.jackson.databind.introspect.DefaultAccessorNamingStrategy.RecordNaming;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Drive extends XboxController {
    public Drive() {
        super(0);
    }
    public Trigger Putter(){
        return new Trigger(this::getXButton);
    }
    public Trigger PutterCorrection(){
        return new Trigger(this::getLeftBumperButton);
    }
}