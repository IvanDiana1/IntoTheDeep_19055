package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Linkage{
    private Servo leftLinkage, rightLinkage;
    private EXTEND_STATES extendState = EXTEND_STATES.CLOSE;
    private double dif = 0.02;
    public Linkage(HardwareMap hwmap, Telemetry telemetry){
        leftLinkage = hwmap.get(Servo.class, HardwareConfig.LeftLinkage);
        rightLinkage = hwmap.get(Servo.class, HardwareConfig.RightLinkage);

        leftLinkage.setPosition(extendState.val+dif);
        rightLinkage.setPosition(extendState.val);
    }

    public enum EXTEND_STATES{
       CLOSE(0.585), PARTIAL_EXTEND(0.22), EXTEND(0.08), MIDDLE(0.32) , HIGHMID(0.27);

       double val;
       EXTEND_STATES(double val){
           this.val=val;
       }

    }

    public void linkageMove() {
        if(extendState==EXTEND_STATES.CLOSE){
            extendState=EXTEND_STATES.EXTEND;
        }
        else {
            extendState=EXTEND_STATES.CLOSE;
        }
        leftLinkage.setPosition(extendState.val+dif);
        rightLinkage.setPosition((extendState.val));
    }
    public void linkageMove(EXTEND_STATES state){
        extendState = state;

        leftLinkage.setPosition(extendState.val+dif);
        rightLinkage.setPosition(extendState.val);
    }


}
