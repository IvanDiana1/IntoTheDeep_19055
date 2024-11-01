package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    protected Servo clawRotation;
    protected Servo clawHold;
    private static HOLD_STATES holdstate = HOLD_STATES.FREE;
    private static ROTATION_STATES rotationState = ROTATION_STATES.LOW;

    public Claw(HardwareMap hwmap){
        clawHold = hwmap.get(Servo.class, HardwareConfig.CLAW_HOLD);
        clawRotation = hwmap.get(Servo.class, HardwareConfig.CLAW_ROTATION);

        clawHold.setPosition(0);
        clawRotation.setPosition(0);
    }
    private enum HOLD_STATES{
        FREE(0),HOLDING(0.45);
        double val;
        HOLD_STATES(double val) {
            this.val = val;
        }
    }
    private enum ROTATION_STATES{
        LOW(0),MID(0.5),HIGH(0.8);
        double val;
        ROTATION_STATES(double val) {
            this.val = val;
        }
    }
    public void clawActivate(){
        if (holdstate==HOLD_STATES.FREE){
            holdstate = HOLD_STATES.HOLDING;
            clawHold.setPosition(holdstate.val);
        }
        else {
            holdstate = HOLD_STATES.FREE;
            clawHold.setPosition(holdstate.val);
        }
    }

    public void clawSetPos(double pos){
        clawHold.setPosition(pos);
        if (pos>0){
            holdstate = HOLD_STATES.HOLDING;
        }
    }
}
