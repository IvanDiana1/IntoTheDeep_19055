package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import kotlin.text.CharDirectionality;

public class Arm {

    private DcMotorEx a_right, a_left;

    public Arm(HardwareMap hardwareMap){

        a_right = hardwareMap.get(DcMotorEx.class, "armRight");
        a_left = hardwareMap.get(DcMotorEx.class, "armLeft");
        a_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        a_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        a_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        a_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        a_right.setDirection(DcMotorEx.Direction.FORWARD);
        a_left.setDirection(DcMotorEx.Direction.REVERSE);


    }
    public void reset(){
        a_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        a_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        a_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        a_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        a_right.setDirection(DcMotorEx.Direction.FORWARD);
        a_left.setDirection(DcMotorEx.Direction.REVERSE);


    }





}
