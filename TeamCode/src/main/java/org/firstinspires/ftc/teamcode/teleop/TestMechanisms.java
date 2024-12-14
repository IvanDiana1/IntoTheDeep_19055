package org.firstinspires.ftc.teamcode.teleop;

import static org.threeten.bp.chrono.JapaneseEra.valueOf;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.util.Arm;
import org.firstinspires.ftc.teamcode.util.Claw;
import org.firstinspires.ftc.teamcode.util.Lifteer;
import org.firstinspires.ftc.teamcode.util.Linkage;
import org.firstinspires.ftc.teamcode.util.Robot;


@TeleOp
public class TestMechanisms extends LinearOpMode {

    public Robot bot;
    public Controller controller1, controller2;
    public Thread uniqueThread;
    double weight = 1;

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bot = new Robot ( hardwareMap,  telemetry);
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);



        waitForStart();

        while(opModeIsActive()&!isStopRequested()){



            bot.drive.setWeightedDrivePower( new Pose2d(
                   - gamepad1.left_stick_y,
                    -gamepad1.left_stick_x *1.4,
                    -gamepad1.right_stick_x)
                    .times(1-(weight*(bot.lifter.currentPos/bot.lifter.MaxRange))));
            //slow drive
            if(controller1.bumperRight.isPressed()){
                weight = 0.5;
            } else
            if(controller1.bumperRight.isReleased()){
                weight = 0;
            }


            if(controller2.triangle.isPressed()&& bot.arm.Target>1){
                bot.linkage.linkageMove();
            }

            if(controller1.circle.isPressed())
            {  bot.claw.clawCatch(); }

            if(controller2.cross.isPressed()&& bot.arm.Target>1)
            {
                bot.claw.clawHRotate();
            }

            if(controller2.circle.isPressed()){
                if (bot.arm.Target==0)
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                else if(bot.arm.Target>1)
                    bot.claw.clawVRotate();
            }

            if(controller2.dpadLeft.isPressed()){
             bot.arm.setTarget(Arm.ARM_STATES.UP.val);

             bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
             bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);


            }
            if(controller2.dpadUp.isPressed()){
                bot.arm.setTarget(Arm.ARM_STATES.UP.val);
                bot.lifter.setTarget(Lifteer.LIFTER_STATES.UP.val);

                bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
                bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                weight = 0.5;


            }
//            if(controller1.dpadDown.isPressed()){
//
//                bot.arm.setTarget(Arm.ARM_STATES.DOWN.val);
//                bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
//                bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
//
////                bot.lifter.setTarget(Lifteer.LIFTER_STATES.DOWN.val);
//             //   weight = 0;
//            }
            if(controller2.dpadDown.isPressed()){

                bot.arm.setTarget(Arm.ARM_STATES.DOWN.val);
                bot.lifter.setTarget(Lifteer.LIFTER_STATES.DOWN.val);

                bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                weight = 0;
            }
            if(controller2.dpadRight.isPressed())
            {   bot.arm.runPid= true;
                bot.arm.setTarget(Arm.ARM_STATES.MIDDLE.val);
            }



//            if(controller1.dpadLeft.isPressed()){
//               bot.arm.runPid= false;
//
//            }


//            if(controller1.triangle.isPressed()){
//                bot.lifter.setTarget(Lifteer.LIFTER_STATES.UP.val);
//                weight = 0.5;
//            }

//            if(controller1.cross.isPressed()){
//
//                bot.lifter.setTarget(Lifteer.LIFTER_STATES.DOWN.val);
//
//                bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
//                weight = 0;
//            }



            bot.lifter.telemetryData();
            bot.arm.telemetryData();

            bot.lifter.update();
            bot.arm.update();
            telemetry.update();
            controller1.update();
            controller2.update();
        }
    }
}
