package org.firstinspires.ftc.teamcode.util;

import android.icu.lang.UCharacter;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive2;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive3;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

public class Robot implements Updateable{
    public Linkage linkage;
     public Claw claw;
     public VoltageSensor voltage_sensor;
     public Lifteer lifter;
     public boolean frozenInitted = false;
     //public AprilTagLibrary apriltaglib = AprilTagGameDatabase.getIntoTheDeepTagLibrary();

     public Robot(){}

     // just overloaded this to not break the clasess in auto
     public Robot(boolean frozenInit){
         FrozenInit();
     }

    public Robot(HardwareMap hwmap, Telemetry telemetry) {
         Init(hwmap, telemetry);
    }

    public SampleMecanumDrive2 drive;
    public void Init(HardwareMap hwmap, Telemetry telemetry ){
        linkage= new Linkage(hwmap,telemetry);
        claw  = new Claw(hwmap,telemetry);
        lifter = new Lifteer(hwmap, telemetry);
        voltage_sensor = hwmap.getAll(VoltageSensor.class).get(0);

        if (!frozenInitted)
            drive = new SampleMecanumDrive2();
        drive.CompleteInit(hwmap);
    }

    public void FrozenInit() {
        drive = new SampleMecanumDrive2();
        frozenInitted = true;
    }


    @Override
    public void update() {
        drive.update();
        lifter.update();
    }
}
