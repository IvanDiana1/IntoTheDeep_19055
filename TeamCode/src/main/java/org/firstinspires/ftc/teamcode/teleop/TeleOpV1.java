package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.util.Robot;

@TeleOp
@Config
public class TeleOpV1 extends LinearOpMode {
    public Robot bot;
    public Controller controller1, controller2;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        bot = new Robot(hardwareMap,telemetry);
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        waitForStart();
   //     bot.arm.lifter.setupLifter();
        while (opModeIsActive() && !isStopRequested()){
            bot.drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);


            if (controller1.cross.isPressed()){
                bot.claw.clawActivate();
            }
            if (controller1.triangle.isPressed()){
                bot.arm.slider.activateLifter();
            }
           /* if (controller1.square.isPressed()){
                bot.arm.lifter.setTarget(10);
            }
            if (controller1.circle.isPressed()){
                bot.arm.lifter.setTarget(450);
            }
            bot.arm.lifter.telemetryData();

            bot.arm.lifter.update();*/
            controller1.update();
            controller2.update();

            telemetry.update();
        }
    }
}
