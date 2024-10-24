package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    public Arm arm;
    public Claw claw;
    public Drive drive;

    public Robot(HardwareMap hwmap, Telemetry telemetry){
        arm = new Arm(hwmap, telemetry);
        claw  = new Claw(hwmap);
        drive = new Drive(hwmap);
    }
}
