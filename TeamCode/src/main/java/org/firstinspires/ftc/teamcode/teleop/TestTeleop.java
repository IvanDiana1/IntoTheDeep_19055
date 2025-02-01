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
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Claw;
import org.firstinspires.ftc.teamcode.util.Lifteer;
import org.firstinspires.ftc.teamcode.util.Linkage;
import org.firstinspires.ftc.teamcode.util.Robot;


@TeleOp
public class TestTeleop extends LinearOpMode {

    public static Robot bot = new Robot();
    public Controller controller1, controller2;
    public double weight = 0.4;
    public Thread uniqueThread = new Thread();
    public static TrajectorySequence place[] = new TrajectorySequence[2];
    private static Drive_Mode driveMode = Drive_Mode.DRIVE_MODE;
    private static final Pose2d observationToWallPos = new Pose2d(13.5, -30.8, Math.toRadians(-180));
    private static final Pose2d wallToObservationPos = new Pose2d(18,0,  Math.toRadians(1));

    private static boolean builtStatically = true;
    private static String errorMessage=" ";


    int lifterInitial = 0;
    int dtlifter=0;

    public boolean isExtended = false;
    public boolean isUp = false;

    enum Drive_Mode {
        DRIVE_MODE,
        AUTO_MODE;
    }

    private static boolean atPickup = true;


    public static void buildTrajectories(){
        place[0] = bot.drive.trajectorySequenceBuilder(observationToWallPos)
                //place first nonpreload specimen on fence

                .addTemporalMarker(0 , 0.1 , ()->
                {   bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                })
                .addTemporalMarker(0,0.31,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
                })

                .addTemporalMarker(0.95,()->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.MIDDLE.val);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);

                })
                .addTemporalMarker(1.92,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
                })





                .setReversed(true)
                .splineToLinearHeading(wallToObservationPos,Math.toRadians(0))
                .build();
        place[1] = bot.drive.trajectorySequenceBuilder(wallToObservationPos)
                .addTemporalMarker(0,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                })
                .addTemporalMarker(0,0.15,()->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.LOWMID.val);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                })
                //place first nonpreload specimen on fence
                .addTemporalMarker(0.25,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);

                })
                .addTemporalMarker(0.7, 0, () ->
                        {
                            bot.linkage.linkageMove(Linkage.EXTEND_STATES.PARTIAL_EXTEND);
                        }
                )


                .lineToLinearHeading(observationToWallPos)
                .build();

    }

    static {
        try{
            bot.FrozenInit();
            buildTrajectories();
        }
        catch(Exception e){
            builtStatically = false;
            errorMessage = e.getMessage();
        }
    }
    public void updateAll(){
        bot.lifter.telemetryData();
        bot.lifter.update();
        telemetry.update();
        controller1.update();
        controller2.update();

    }
    @Override
     public void runOpMode() throws InterruptedException{

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        bot.Init(hardwareMap,telemetry);
        bot.lifter.setTarget(Lifteer.autoPos);

        if (!builtStatically) {

            telemetry.addLine("Trajectories COULDNT be built statically");
            telemetry.update();

            buildTrajectories();

            telemetry.addLine("que comience el espectaculo");
            telemetry.addLine(errorMessage);
            telemetry.update();
        }
        else{
            telemetry.addLine("Trajectories SUCCESSFULLY built statically: ");
            telemetry.update();

        }

        waitForStart();
        bot.lifter.reset_motors();
        bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);

        while(opModeIsActive() && !isStopRequested()){


            //drive
            if (driveMode == Drive_Mode.AUTO_MODE){
                controller1.update();
                bot.drive.update();
                bot.lifter.update();
                bot.lifter.telemetryData();
                telemetry.update();

                if (controller1.square.isPressed()) {
                    bot.drive.breakFollowing();
                    driveMode = Drive_Mode.DRIVE_MODE;
                }
                if(controller1.bumperLeft.isPressed()){
                    bot.claw.clawCatch();
                }
                if (!bot.drive.isBusy())
                    driveMode = Drive_Mode.DRIVE_MODE;
                continue;
            }

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

            if(controller1.dpadDown.isPressed()){
                bot.lifter.setTarget(Lifteer.LIFTER_STATES.AGATATED.val);
            }

            if(controller1.triangle.isPressed()){
                bot.claw.clawVRotate(Claw.VERTICAL_STATES.LOWMID);
            }
            if (controller1.cross.isPressed()){
                atPickup = true;
            }
            if (controller1.circle.isPressed()){
                if (atPickup) {
                    bot.drive.setPoseEstimate(observationToWallPos);
                    bot.drive.followTrajectorySequenceAsync(place[0]);
                    atPickup = false;
                }
                else {
                    bot.drive.setPoseEstimate(wallToObservationPos);
                    bot.drive.followTrajectorySequenceAsync(place[1]);
                    atPickup = true;
                }
                driveMode = Drive_Mode.AUTO_MODE;
            }

            //close linkage
            if(controller2.triangle.isPressed()) {
                uniqueThread.interrupt();
                if (bot.lifter.Target == Lifteer.LIFTER_STATES.UP.val || bot.lifter.Target == Lifteer.LIFTER_STATES.MIDDLE.val ) {
                    bot.linkage.linkageMove();
                    isExtended = !isExtended;
                } else
                if( bot.lifter.Target == Lifteer.LIFTER_STATES.LOWMID.val){
                      if(isExtended) {

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
                bot.lifter.setTarget(Lifteer.LIFTER_STATES.MIDDAL.val);
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

            updateAll();

        }

     bot.lifter.setAutoPos(0);


    }

}
