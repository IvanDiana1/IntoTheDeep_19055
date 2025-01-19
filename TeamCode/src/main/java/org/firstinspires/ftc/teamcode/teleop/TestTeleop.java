package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.util.Claw;
import org.firstinspires.ftc.teamcode.util.Lifteer;
import org.firstinspires.ftc.teamcode.util.Linkage;
import org.firstinspires.ftc.teamcode.util.Robot;


@TeleOp
public class TestTeleop extends LinearOpMode {




    public Robot bot = new Robot();
    public Controller controller1, controller2;
    public double weight = 0.4;
    public Thread uniqueThread = new Thread();


    int lifterInitial = 0;
    int dtlifter=0;

    public boolean isExtended = false;
    public boolean isUp = false;
    @Override
     public void runOpMode() throws InterruptedException{

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        bot.Init(hardwareMap,telemetry);
        bot.lifter.setTarget(Lifteer.autoPos);
        waitForStart();
        bot.lifter.reset_motors();
//        bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);

        while(opModeIsActive() && !isStopRequested()){

            //drive

            bot.drive.setWeightedDrivePower( new Pose2d(
                    - gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x)
                    .times(1- weight*((double)max(1,bot.lifter.currentPos)/Lifteer.LIFTER_STATES.UP.val)));
            //slow drive
            if(controller1.bumperRight.isPressed()){
                weight = 0.7;
            } else
            if(controller1.bumperRight.isReleased()){
                weight = 0.4;
            }

// open-close claw

            if(controller1.bumperLeft.isPressed()){
                bot.claw.clawCatch();
            }

            if(controller1.square.isPressed()){
                bot.lifter.setTarget(Lifteer.LIFTER_STATES.SPECIMEN.val);
            }

            if(controller1.dpadDown.isPressed()){
                bot.lifter.setTarget(Lifteer.LIFTER_STATES.AGATATED.val);
            }

            if(controller1.circle.isPressed()){
                bot.claw.clawVRotate(Claw.VERTICAL_STATES.LOWMID);
            }






            //collect from submersible

//            if(controller2.triangle.isPressed()){
//
//
//
//                uniqueThread.interrupt();
//                uniqueThread = new Thread(() -> {
//                    if (!isExtended) {
//                        if(!isUp){
//                        bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
//                        sleep(300);
//                        if (Thread.currentThread().isInterrupted()) {
//                            return;
//                        }}
//                        bot.linkage.linkageMove(Linkage.EXTEND_STATES.PARTIAL_EXTEND);
//                        isExtended = true;
//                        bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
//                        sleep(400);
//                        if (Thread.currentThread().isInterrupted()) {
//                            return;
//                        }
//                        bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);
//                    }
//                }  );
//                uniqueThread.start();
//            }


            //close linkage
            if(controller2.triangle.isPressed()) {
                uniqueThread.interrupt();
                if (bot.lifter.Target == Lifteer.LIFTER_STATES.UP.val || bot.lifter.Target == Lifteer.LIFTER_STATES.MIDDLE.val ) {
                    bot.linkage.linkageMove();
                    isExtended = !isExtended;
                } else
                if( bot.lifter.Target == Lifteer.LIFTER_STATES.LOWMID.val){
                      if(isExtended) {
                          //bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);

                          sleep(300);
                          if (Thread.currentThread().isInterrupted()) {
                              return;
                          }
                          bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                          isExtended = false;
                      }
                      else
                      {
                          bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                          if (Thread.currentThread().isInterrupted()) {
                              return;
                          }
                          bot.linkage.linkageMove(Linkage.EXTEND_STATES.PARTIAL_EXTEND);
                          isExtended = true;
                      }


                }


                else {
                    uniqueThread = new Thread(() -> {
                        if (isExtended) {
                            bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                            bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                            bot.claw.clawHRotate(Claw.HORIZONTAL_STATES.PARALEL);
                            sleep(300);
                            if (Thread.currentThread().isInterrupted()) {
                                return;
                            }

                            isExtended = false;

                            sleep(300);
                            if (Thread.currentThread().isInterrupted()) {
                                return;
                            }

                        } else {
                            bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                            sleep(300);
                            if (Thread.currentThread().isInterrupted()) {
                                return;
                            }
                            bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
                            isExtended = true;

                        }
                    }
                    );

                    uniqueThread.start();
                }
            }

//horizontal movement
            if(controller2.cross.isPressed()){

                if(isExtended){
                    bot.claw.clawHRotate();}



            }

            if(controller2.circle.isPressed()){
                if (bot.lifter.Target<Lifteer.LIFTER_STATES.LOWMID.val+260 && bot.lifter.Target>Lifteer.LIFTER_STATES.LOWMID.val-260 && bot.claw.vState == Claw.VERTICAL_STATES.MIDDLE)
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
                else
                    bot.claw.clawVRotate();
            }

            if (Math.abs(controller2.leftStickY)>0.2){
                if (controller2.leftStickY<0)
                    dtlifter+=4;
                else
                    dtlifter-=4;
                if (Math.abs(dtlifter)==4)
                    lifterInitial = bot.lifter.currentPos;
                bot.lifter.setTarget(lifterInitial+dtlifter);
            }
            else {
                lifterInitial = 0;
                dtlifter=0;
            }

            if(controller2.dpadUp.isPressed()){

             bot.lifter.setTarget(Lifteer.LIFTER_STATES.UP.val);
             isUp = true;
            }

            if(controller2.dpadDown.isPressed()){
                bot.lifter.setTarget(Lifteer.LIFTER_STATES.DOWN.val);
                sleep(400);
                bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                isExtended = false;
                isUp = false;
            }

            if( controller2.dpadRight.isPressed()){
                bot.lifter.setTarget(Lifteer.LIFTER_STATES.MIDDLE.val);
                isUp = true;
            }
            if (controller2.dpadLeft.isPressed()){
                bot.lifter.setTarget(Lifteer.LIFTER_STATES.LOWMID.val);
                isUp = true;
            }
            if(controller2.bumperRight.isPressed() ){
                bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
            }

            if(controller2.bumperLeft.isPressed()){
                bot.lifter.setTarget(Lifteer.LIFTER_STATES.AGATATED.val);
            }

            bot.lifter.telemetryData();
            bot.lifter.update();
            telemetry.update();
            controller1.update();
            controller2.update();

        }

     bot.lifter.setAutoPos(0);


    }

}
