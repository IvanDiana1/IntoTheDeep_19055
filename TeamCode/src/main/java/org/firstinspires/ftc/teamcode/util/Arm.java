package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {
    public Lifter lifter;
    public Slider slider;
    public Arm(HardwareMap hwmap, Telemetry telemetry){
        lifter = new Lifter(hwmap,telemetry);
        slider = new Slider(hwmap);
    }
}
