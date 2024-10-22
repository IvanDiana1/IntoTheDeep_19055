package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class Robot {

public SampleMecanumDrive drive;
public Claw claw;
public Lifter lifter;
public ClawRotation clawRotation;
public void init ( @NonNull HardwareMap hardwareMap){

    drive= new SampleMecanumDrive(hardwareMap);
    claw = new Claw(hardwareMap);
    lifter = new Lifter(hardwareMap);
    clawRotation = new ClawRotation(hardwareMap);

}


}
