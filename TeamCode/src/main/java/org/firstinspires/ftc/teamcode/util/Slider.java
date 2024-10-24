package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Slider {
    protected Servo slider;
    private static SLIDER_STATES slider_state = SLIDER_STATES.NEUTRAL;
    public Slider(HardwareMap hwmap){
        slider = hwmap.get(Servo.class, HardwareConfig.SLIDER_SERVO);

        slider.setPosition(SLIDER_STATES.NEUTRAL.val);
    }
    private enum SLIDER_STATES{
        NEUTRAL(0.4), EXTENDED(0);
        double val;
         SLIDER_STATES(double val) {
            this.val = val;
        }
    }
    public void setPosition(SLIDER_STATES pos){
        slider.setPosition(pos.val);
        slider_state = pos;
    }
    public void activateLifter(){
        if (slider_state==SLIDER_STATES.NEUTRAL)
            slider_state = SLIDER_STATES.EXTENDED;

        else
            slider_state = SLIDER_STATES.NEUTRAL;

        slider.setPosition(slider_state.val);

    }

}
