package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lifter {

    private Servo lifter;

    public Lifter (@NonNull HardwareMap hardwareMap){

        lifter = hardwareMap.get(Servo.class, "lifter");
        lifter.setPosition(0.45);

    }

    public void LifterExtend(){

        lifter.setPosition(0);
    }

    public void LifterClose(){

        lifter.setPosition(0.45);

    }

}
