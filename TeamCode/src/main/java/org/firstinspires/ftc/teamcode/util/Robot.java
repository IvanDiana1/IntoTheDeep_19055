package org.firstinspires.ftc.teamcode.util;

import android.icu.lang.UCharacter;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive3;

public class Robot {
    public Linkage linkage;
     public Claw claw;
     public VoltageSensor voltage_sensor;
     public Lifteer lifter;

    public SampleMecanumDrive3 drive;
    public void Init(HardwareMap hwmap, Telemetry telemetry ){
        linkage= new Linkage(hwmap);
        claw  = new Claw(hwmap,telemetry);
        drive = new SampleMecanumDrive3(hwmap);
        lifter = new Lifteer(hwmap, telemetry);
        voltage_sensor = hwmap.getAll(VoltageSensor.class).get(0);
    }
}
