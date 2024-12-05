package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw {
    private Servo clawHold, clawHRot, clawVRot;
    private Telemetry telemetry;
    private static HOLD_STATES holdState = HOLD_STATES.HOLD;
    private static VERTICAL_STATES vState = VERTICAL_STATES.DOWN;
    private static HORIZONTAL_STATES hState = HORIZONTAL_STATES.PARALEL;

    public Claw(HardwareMap hwmap, Telemetry telemetry){
        clawHold = hwmap.get(Servo.class, HardwareConfig.ClawHold);
        clawHRot = hwmap.get(Servo.class, HardwareConfig.ClawHRotation);
        clawVRot = hwmap.get(Servo.class, HardwareConfig.ClawVRotation);

        clawHold.setPosition(holdState.val);
        clawHRot.setPosition(hState.val);
        clawVRot.setPosition(vState.val);

    }
    private enum HOLD_STATES{
        HOLD(1),RELEASE(0);
        double val;
        HOLD_STATES(double val) {
            this.val = val;
        }
    }
    private enum HORIZONTAL_STATES {
        PARALEL(0.5),PERPENDICULAR(0), REVERESED(1);
        double val;
        HORIZONTAL_STATES(double val) {
            this.val = val;
        }
    }
    private enum VERTICAL_STATES{
        UP(1),DOWN(0);
        double val;
        VERTICAL_STATES(double val) {
            this.val = val;
        }
    }
    public void clawCatch(){
        if (holdState==HOLD_STATES.HOLD)
            holdState = HOLD_STATES.RELEASE;
        else
            holdState = HOLD_STATES.HOLD;
        clawHold.setPosition(holdState.val);
    }
    public void clawHRotate(){
        if (hState==HORIZONTAL_STATES.PARALEL)
            hState = HORIZONTAL_STATES.PERPENDICULAR;
        else
            hState = HORIZONTAL_STATES.PARALEL;
        clawHRot.setPosition(holdState.val);
    }
    public void clawVRotate(){
        if (vState==VERTICAL_STATES.UP)
            vState = VERTICAL_STATES.DOWN;
        else
            vState = VERTICAL_STATES.UP;
        clawVRot.setPosition(vState.val);
    }

    public void printAllData(){
    }
}
