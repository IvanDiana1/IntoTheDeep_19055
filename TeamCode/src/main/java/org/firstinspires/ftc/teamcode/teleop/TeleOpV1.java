package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.util.Lifter;
import org.firstinspires.ftc.teamcode.util.Robot;

@TeleOp
@Config
public class TeleOpV1 extends LinearOpMode {
    public Robot bot;
    public Controller controller1, controller2;

    public double weight = 1;

    public Thread uniqueThread = new Thread();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        bot = new Robot(hardwareMap,telemetry);
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        waitForStart();


//        bot.arm.lifter.setupLifter();
        while (opModeIsActive() && !isStopRequested()){
            //drive
            bot.drive.setWeightedDrivePower( new Pose2d(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x)
                    .times(weight));
            //slow drive
            if(controller1.bumperRight.isPressed()){
                    weight = 0.5;
            } else
                if(controller1.bumperRight.isReleased()){
                    weight = 1;
                }

            if (controller1.cross.isPressed()){
                bot.claw.clawActivate();
            }
            if (controller1.triangle.isPressed()){
                bot.arm.slider.activateLifter();
            }
            if (controller1.dpadDown.isPressed()){
                bot.arm.lifter.setTarget(0);
            }

            if (controller1.dpadUp .isPressed()){
                bot.arm.lifter.setTarget(220);
            }
            if(controller1.dpadRight.isPressed()){
                bot.arm.lifter.setTarget(125);
            }
            bot.arm.lifter.telemetryData();

            bot.arm.lifter.update();
            controller1.update();
            controller2.update();

            telemetry.update();
        }
    }
}
