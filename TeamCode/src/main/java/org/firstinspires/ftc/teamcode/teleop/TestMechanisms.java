package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.util.Robot;


@TeleOp
public class TestMechanisms extends LinearOpMode {

    public Robot bot;
    public Controller controller1, controller2;
    public Telemetry telemetry;

    @Override
    public void runOpMode() throws InterruptedException{
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bot = new Robot ( hardwareMap,  telemetry);
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);



        waitForStart();

        while(opModeIsActive()&!isStopRequested()){

            if(controller2.triangle.isPressed()){
                bot.linkage.linkageMove();
            }

            if(controller2.circle.isPressed())
            {  bot.claw.clawCatch(); }

            if(controller2.dpadUp.isPressed())
            {
                bot.claw.clawHRotate();
            }

            if(controller2.dpadDown.isPressed()){
                bot.claw.clawVRotate();
            }

            controller1.update();
            controller2.update();
        }
    }
}
