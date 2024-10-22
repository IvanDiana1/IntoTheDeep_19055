package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    private Servo claw;
    public Claw (HardwareMap hardwareMap){
    claw = hardwareMap.get(Servo.class,"claw");
   // claw.setDirection(Servo.Direction.FORWARD);
    claw.setPosition(1);
//
claw.setDirection(Servo.Direction.REVERSE);
    }

    public void ClawOpen(){
        claw.setPosition(0.0);
    }
    public void ClawClose(){
        claw.setPosition(0.8);
    }
}