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
    double weight = 0;
    int lifterInitial = 0;
    int dtlifter=0;

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        bot = new Robot (hardwareMap, telemetry);



        waitForStart();

        while(opModeIsActive()&!isStopRequested()){


            bot.drive.setWeightedDrivePower( new Pose2d(
                   - gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x)
                    .times(1-(weight*(bot.lifter.currentPos/(bot.lifter.Target+1)))));
            //slow drive
            if(controller1.bumperRight.isPressed()){
                weight = 0.5;
            } else
            if(controller1.bumperRight.isReleased()){
                weight = 0;
            }

            if (Math.abs(controller2.leftStickY)>0.2){
                if (controller2.leftStickY<0)
                    dtlifter+=4;
                else
                    dtlifter-=4;
                if (Math.abs(dtlifter)==4)
                    lifterInitial = bot.arm.getArmPos();

                bot.arm.setTarget(lifterInitial+dtlifter);
            }
            else {
                lifterInitial = 0;
                dtlifter=0;
            }


            if(controller2.triangle.isPressed()&& bot.arm.Target>1){
                bot.linkage.linkageMove();
                bot.claw.clawHRotate(Claw.HORIZONTAL_STATES.PARALEL);
                bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
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
                bot.claw.clawHRotate(Claw.HORIZONTAL_STATES.PARALEL);
                bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                weight = 0;
            }
            if(controller2.dpadRight.isPressed()){
                bot.arm.setTarget(Arm.ARM_STATES.MIDDLE.val);
              //  bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);

            }

            if(controller2.bumperRight.isPressed())
                bot.claw.clawVRotate(Claw.VERTICAL_STATES.INIT );


//            if(controller1.dpadLeft.isPressed()){
//               bot.arm.runPid= false;
//
//            }




            if(controller2.bumperLeft.isPressed()){
                if (bot.lifter.Target== Lifteer.LIFTER_STATES.UP.val|| bot.lifter.Target== Lifteer.LIFTER_STATES.UP.val+1460)
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.MIDDLE.val);
                else
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.UP.val+1460);

                weight = 0;
            }





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
