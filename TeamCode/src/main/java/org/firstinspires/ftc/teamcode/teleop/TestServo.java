package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Controller;

import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestServo extends LinearOpMode {
    public Servo left;
    public Servo right;
    Controller controller1;

    @Override
    public void runOpMode() throws  InterruptedException{
        left = hardwareMap.get(Servo.class,"left");
        //right = hardwareMap.get(Servo.class,"right");
        left.setPosition(0);
        //right.setPosition(0);
        controller1=new Controller(gamepad1);

        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            if(controller1.triangle.isPressed()){
                left.setPosition(0);
                //right.setPosition(0);
            }
            if(controller1.square.isPressed()){
                left.setPosition(0.4);
            //    right.setPosition(0.55);
            }
            controller1.update();
        }
    }
}
