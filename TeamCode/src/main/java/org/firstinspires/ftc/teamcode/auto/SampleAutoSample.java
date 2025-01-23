package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;


import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive3;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Claw;
import org.firstinspires.ftc.teamcode.util.Lifteer;
import org.firstinspires.ftc.teamcode.util.Linkage;
import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous
public class SampleAutoSample extends LinearOpMode {
    Robot bot = new Robot();

    public static TrajectorySequence startPhase[] = new TrajectorySequence[1],place[] = new TrajectorySequence[4] , parking[] = new TrajectorySequence[1];
    public Thread uniqueThread;


    private void drop(){
        bot.linkage.linkageMove(Linkage.EXTEND_STATES.PARTIAL_EXTEND);
        sleep(200);
        bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
        sleep(150);
        bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
        sleep(100);
        bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
        sleep(200);
        bot.linkage.linkageMove(Linkage.EXTEND_STATES.MIDDLE);
        sleep(100);
        bot.lifter.setTarget(Lifteer.LIFTER_STATES.DOWN.val);

    }

    private void pickUp(Claw.HORIZONTAL_STATES state){
        bot.linkage.linkageMove(Linkage.EXTEND_STATES.HIGHMID);
        sleep(100);
        bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);
        bot.claw.clawHRotate(state);
        sleep(400);
        bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
        sleep(300);
        bot.claw.clawHRotate(Claw.HORIZONTAL_STATES.PARALEL);
        bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
        bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
        sleep(200);
        bot.claw.clawHRotate(Claw.HORIZONTAL_STATES.PARALEL);
        bot.lifter.setTarget(Lifteer.LIFTER_STATES.UP.val);

    }

    public void buildTrajectories(){
        startPhase[0] = bot.drive.trajectorySequenceBuilder( new Pose2d(-2 , 0 , Math.toRadians(90)))
                .addSpatialMarker( new Vector2d( 0 , 0) , () ->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.UP.val);
                })
                .addTemporalMarker(0.48 ,0,  () ->
                {
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.PARTIAL_EXTEND);

                })
                .addTemporalMarker(0.5,0.1,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                })
                .addTemporalMarker(0.5,0.3, () ->
                {
                    bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
                })
                .addTemporalMarker(0.55,0.25 , () ->
                {
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.DOWN.val);
                })

                .addTemporalMarker(0.55,0.3 , () ->
                {
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.HIGHMID);
                })



                .lineToLinearHeading( new Pose2d( 16, 37, Math.toRadians(150)))

                .splineToLinearHeading(new Pose2d(23,26,Math.toRadians(30)),Math.toRadians(0),
                        SampleMecanumDrive3.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive3.getAccelerationConstraint(10))

                .build();

        place[0] = bot.drive.trajectorySequenceBuilder( startPhase[0].end())
                //vere ce plm e theme-ul asta ajutor
                .lineToLinearHeading(new Pose2d(15, 38, Math.toRadians(160)))

                .build();

        place[1] = bot.drive.trajectorySequenceBuilder(place[0].end())
                .setAccelConstraint(new ProfileAccelerationConstraint(7))
                .splineToLinearHeading(new Pose2d(19.8,43.5,Math.toRadians(0)),Math.toRadians(0))
                .build();

        place[2] = bot.drive.trajectorySequenceBuilder(place[1].end())
                .lineToLinearHeading(new Pose2d(13.5 , 34.5, Math.toRadians(-215)))
                .build();

        place[3] = bot.drive.trajectorySequenceBuilder(place[2].end())
                .addTemporalMarker(0.1,0,()->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.LOWMID.val);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);
                    bot.claw.clawHRotate(Claw.HORIZONTAL_STATES.PERPENDICULARLeft);
                })
                .addTemporalMarker(0.3,0.2,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.HIGHMID);
                })

                //start lowering lifter
                .addTemporalMarker(0.4,0,()->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.DOWN.val);
                })

                //collect last sample from spikemark
                .addTemporalMarker(0.4,0.2,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                })

                .addTemporalMarker(0.4,0.45,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.LOWMID);
                })

                //raise last sample from spikemark
                .addTemporalMarker(0.6,0,()->{
                    bot.claw.clawHRotate(Claw.HORIZONTAL_STATES.PARALEL);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
                })
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(34.3, 39.5, Math.toRadians(93)),Math.toRadians(0),
                        SampleMecanumDrive3.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive3.getAccelerationConstraint(10))

                //leave sample in observation area
                .addTemporalMarker(0.6,0.1,()->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.UP.val);
                })
                .addTemporalMarker(0.92,0.2,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                })
                .addTemporalMarker(0.92,0.3,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
                })
                .addTemporalMarker(0.92,0.6,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);

                })
                .splineToLinearHeading(new Pose2d(11, 34.7, Math.toRadians(-210)),Math.toRadians(0),
                        SampleMecanumDrive3.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive3.getAccelerationConstraint(15))
                .waitSeconds(0.4)
                .build();

        parking[0] = bot.drive.trajectorySequenceBuilder(place[3].end())
                .addTemporalMarker(0.35,0.1, () -> {
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.MIDDLE.val-20);
                } )
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(50,14.5,Math.toRadians(-90)),Math.toRadians(200),
                        SampleMecanumDrive3.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive3.getAccelerationConstraint(15))
                .build();

    }


    @Override
    public void runOpMode() throws InterruptedException {
        bot.Init(hardwareMap, telemetry);
        bot.lifter.set_motorsResetable(false);
        bot.lifter.setAutoPos(0);


        buildTrajectories();
        telemetry.addLine("que comience el espectaculo");
        telemetry.update();

        waitForStart();

        if(isStopRequested()) return;

        bot.drive.setPoseEstimate(new Pose2d(-2,0,Math.toRadians(90)));
        bot.drive.followTrajectorySequenceAsync(startPhase[0]);

        while (((bot.lifter.isBusy()||bot.drive.isBusy()) && !isStopRequested()) ) {
            bot.drive.update();
            telemetry.addData("er", bot.drive.getLastError());
            bot.lifter.update();
            telemetry.update();
        }

        pickUp(Claw.HORIZONTAL_STATES.TILTED);

        bot.drive.followTrajectorySequenceAsync(place[0]);

        while ( (bot.lifter.isBusy() || bot.drive.isBusy()) && !isStopRequested() ) {
            bot.drive.update();
            telemetry.addData("ISBUSY:",bot.lifter.isBusy());
            bot.lifter.update();
            telemetry.update();
        }

        drop();

        bot.drive.followTrajectorySequenceAsync(place[1]);

        while (((bot.lifter.isBusy()||bot.drive.isBusy()) && !isStopRequested()) ) {
            bot.drive.update();
            telemetry.addData("er", bot.drive.getLastError());
            bot.lifter.update();
            telemetry.update();
        }

        pickUp(Claw.HORIZONTAL_STATES.PARALEL);

        bot.drive.followTrajectorySequenceAsync(place[2]);

        while (((bot.lifter.isBusy()||bot.drive.isBusy()) && !isStopRequested()) ) {
            bot.drive.update();
            telemetry.addData("er", bot.drive.getLastError());
            bot.lifter.update();
            telemetry.update();
        }

        drop();

        bot.drive.followTrajectorySequenceAsync(place[3]);

        while (((bot.lifter.isBusy()||bot.drive.isBusy()) && !isStopRequested()) ) {
            bot.drive.update();
            telemetry.addData("er", bot.drive.getLastError());
            bot.lifter.update();
            telemetry.update();
        }

        bot.drive.followTrajectorySequenceAsync(parking[0]);

        while (((bot.lifter.isBusy()||bot.drive.isBusy()) && !isStopRequested()) ) {
            bot.drive.update();
            telemetry.addData("er", bot.drive.getLastError());
            bot.lifter.update();
            telemetry.update();
        }

        bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
        sleep(400);
        bot.lifter.setTarget(Lifteer.LIFTER_STATES.DOWN.val);
        sleep(50);
        uniqueThread = new Thread(() -> {
            while (bot.lifter.isBusy() && !uniqueThread.isInterrupted()){
                bot.lifter.update();
            }
        });
        uniqueThread.start();

        sleep(55);

        uniqueThread.interrupt();
        bot.lifter.float_motors();

        sleep(300);

        bot.lifter.update();
        bot.lifter.setAutoPos(bot.lifter.currentPos);
        sleep(500);

        telemetry.addLine("c'est fini");
        telemetry.addData("timp: ",getRuntime());
        telemetry.update();
    };

}