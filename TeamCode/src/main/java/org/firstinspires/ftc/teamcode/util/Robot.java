package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive3;

public class Robot {
    public Arm arm;
    public Claw claw;

    public SampleMecanumDrive3 drive;
    public Robot(HardwareMap hwmap, Telemetry telemetry){
        arm = new Arm(hwmap, telemetry);
        claw  = new Claw(hwmap);
        drive = new SampleMecanumDrive3(hwmap);
    }
}
