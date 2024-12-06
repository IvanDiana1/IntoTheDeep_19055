package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class Linkage {
    private Servo leftLinkage, rightLinkage;
    private EXTEND_STATES extendState = EXTEND_STATES.CLOSE;
    private double dif = 0.04;
    boolean isExtended;
    public Linkage(HardwareMap hwmap ){

        leftLinkage = hwmap.get(Servo.class, HardwareConfig.LeftLinkage);
        rightLinkage = hwmap.get(Servo.class, HardwareConfig.RightLinkage);

        leftLinkage.setPosition(extendState.val+dif);
        rightLinkage.setPosition(extendState.val);
        isExtended = false;
    }

    private enum EXTEND_STATES{
       CLOSE(0.55), EXTEND(0);

       double val;
       EXTEND_STATES(double val){
           this.val=val;
       }

    }

    public void linkageMove() {
        if(isExtended==false){
            extendState=EXTEND_STATES.EXTEND;
            isExtended = true;
        }
        else {extendState=EXTEND_STATES.CLOSE;
            isExtended=false;}
        leftLinkage.setPosition(extendState.val+dif);
        rightLinkage.setPosition((extendState.val));
    }







}
