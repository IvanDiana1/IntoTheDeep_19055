package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw {
    private Servo clawHold, clawHRot, clawVRot;
    private Telemetry telemetry;
    private  HOLD_STATES holdState = HOLD_STATES.HOLD;
    private  VERTICAL_STATES vState = VERTICAL_STATES.INIT;
    private  HORIZONTAL_STATES hState = HORIZONTAL_STATES.PARALEL;
    boolean isClawClosed;

    public Claw(HardwareMap hwmap, Telemetry telemetry){
        clawHold = hwmap.get(Servo.class, HardwareConfig.ClawHold);
        clawHRot = hwmap.get(Servo.class, HardwareConfig.ClawHRotation);
        clawVRot = hwmap.get(Servo.class, HardwareConfig.ClawVRotation);

        clawHold.setPosition(HOLD_STATES.HOLD.val);
        isClawClosed = true;
        clawHRot.setPosition(HORIZONTAL_STATES.PARALEL.val);
        clawVRot.setPosition(VERTICAL_STATES.INIT.val);

    }
    private enum HOLD_STATES{
        HOLD(0.05),RELEASE(0.37);
        double val;
        HOLD_STATES(double val) {
            this.val = val;
        }
    }
    private enum HORIZONTAL_STATES {
        PARALEL(0),PERPENDICULAR(0.5), REVERESED(1);
        double val;
        HORIZONTAL_STATES(double val) {
            this.val = val;
        }
    }
    private enum VERTICAL_STATES{
      INIT(0.2) , MIDDLE (0.5),  UP(0.5),DOWN(1);
        double val;
        VERTICAL_STATES(double val) {
            this.val = val;
        }
    }
    public void clawCatch(){
        if (isClawClosed==true)
        { holdState = HOLD_STATES.RELEASE;
            isClawClosed = false;}
        else
        if(isClawClosed==false)
        {   holdState = HOLD_STATES.HOLD;
            isClawClosed = true;}
        clawHold.setPosition(holdState.val);
    }
    public void clawHRotate(){
        // paralel -> perpendicular - > reversed
        if (hState==HORIZONTAL_STATES.PARALEL)
        {   hState = HORIZONTAL_STATES.PERPENDICULAR; }

        else if(hState==HORIZONTAL_STATES.PERPENDICULAR)
        {   hState = HORIZONTAL_STATES.REVERESED;}

        else
            if(hState == HORIZONTAL_STATES.REVERESED)
                hState = HORIZONTAL_STATES.PARALEL;

        clawHRot.setPosition(hState.val);
    }
    public void clawVRotate(){
        if (vState==VERTICAL_STATES.UP || vState== VERTICAL_STATES.INIT )
            vState = VERTICAL_STATES.DOWN;
        else
            if(vState == VERTICAL_STATES.DOWN)
            { vState = VERTICAL_STATES.UP;}


        clawVRot.setPosition(vState.val);
    }

    public void printAllData(){
    }
}
