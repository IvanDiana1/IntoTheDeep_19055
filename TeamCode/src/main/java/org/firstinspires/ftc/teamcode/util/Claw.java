package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw {
    private Servo clawHold, clawHRot, clawVRot;
    private Telemetry telemetry;
    public  HOLD_STATES holdState = HOLD_STATES.HOLD;
    public  VERTICAL_STATES vState = VERTICAL_STATES.UP;
    public  HORIZONTAL_STATES hState = HORIZONTAL_STATES.PARALEL;

    public Claw(HardwareMap hwmap, Telemetry telemetry){
        clawHold = hwmap.get(Servo.class, HardwareConfig.ClawHold);
        clawHRot = hwmap.get(Servo.class, HardwareConfig.ClawHRotation);
        clawVRot = hwmap.get(Servo.class, HardwareConfig.ClawVRotation);

        clawHold.setPosition(holdState.val);
        clawHRot.setPosition(hState.val);
        clawVRot.setPosition(vState.val);

    }
    public enum HOLD_STATES{
        HOLD(0.42),RELEASE(0.15) , HOLD_STRONGER(0.5);
        double val;
        HOLD_STATES(double val) {
            this.val = val;
        }
    }
    public enum HORIZONTAL_STATES {
        PARALEL(0),PERPENDICULAR(0.5), REVERESED(1),TILTED(0.2);
        double val;
        HORIZONTAL_STATES(double val) {
            this.val = val;
        }
    }
    public enum VERTICAL_STATES{
      UP(1),DOWN(0.22),  LOWMID(0.3),  HIGHMID(0.75),   MIDDLE(0.65);
        double val;
        VERTICAL_STATES(double val) {
            this.val = val;
        }
    }
    public void clawCatch(){
        if (holdState==HOLD_STATES.HOLD)
            holdState = HOLD_STATES.RELEASE;
        else if(holdState==HOLD_STATES.RELEASE)
           holdState = HOLD_STATES.HOLD;
        clawHold.setPosition(holdState.val);
    }
    public void clawHRotate(){
        // paralel -> perpendicular - > reversed
        if (hState==HORIZONTAL_STATES.PARALEL)
            hState = HORIZONTAL_STATES.PERPENDICULAR;

        else if(hState==HORIZONTAL_STATES.PERPENDICULAR)
            hState = HORIZONTAL_STATES.PARALEL;

        clawHRot.setPosition(hState.val);
    }
    public void clawVRotate(){
        if (vState==VERTICAL_STATES.UP|| vState == VERTICAL_STATES.MIDDLE)
            vState = VERTICAL_STATES.DOWN;
        else if (vState==VERTICAL_STATES.DOWN || vState == VERTICAL_STATES.LOWMID)
            vState = VERTICAL_STATES.UP;


        clawVRot.setPosition(vState.val);
    }

    public void clawCatch(HOLD_STATES state){
        holdState = state;
        clawHold.setPosition(holdState.val);
    }

    public void clawVRotate(VERTICAL_STATES state){
        vState = state;
        clawVRot.setPosition(vState.val);
    }

    public void clawHRotate(HORIZONTAL_STATES state){
        hState = state;
        clawHRot.setPosition(hState.val);
    }

    public void printAllData(){
    }
}
