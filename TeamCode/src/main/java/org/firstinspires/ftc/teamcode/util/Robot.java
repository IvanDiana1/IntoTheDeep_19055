package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive3;

public class Robot {
    public Linkage linkage;
     public Claw claw;
     public Arm arm;
     public Lifteer lifter;

    public SampleMecanumDrive3 drive;
    public Robot(HardwareMap hwmap, Telemetry telemetry ){
        linkage= new Linkage(hwmap);
        claw  = new Claw(hwmap,telemetry);
        drive = new SampleMecanumDrive3(hwmap);
        arm = new Arm(hwmap, telemetry, linkage);
        lifter = new Lifteer(hwmap, telemetry);
    }
}
