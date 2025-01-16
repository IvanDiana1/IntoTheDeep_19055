package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Claw;
import org.firstinspires.ftc.teamcode.util.Lifteer;
import org.firstinspires.ftc.teamcode.util.Linkage;
import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous
public class SpecimenRemakeRemakeRemake extends LinearOpMode {
    Robot bot = new Robot();
    public static TrajectorySequence park[] = new TrajectorySequence[2] ,startPhase[] = new TrajectorySequence[1], spikeMarks[] = new TrajectorySequence[4] , place[] = new TrajectorySequence[3] , collect[] = new TrajectorySequence[1];
    public static Pose2d marksPosFence[] = new  Pose2d[3] ;
    public static Pose2d marksPosWall[] = new Pose2d[3];

    public void buildTrajectories() {
        startPhase[0] = bot.drive.trajectorySequenceBuilder(new Pose2d(0.1, 0, Math.toRadians(0)))
                .addSpatialMarker(new Vector2d(0.1, 0), () -> {
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.SPECIMEN.val);
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.PARTIAL_EXTEND);

                })
                .addTemporalMarker(0.5,0.2, () -> {
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);
                })
                .addTemporalMarker(0.5,0.5,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
                })
                .addTemporalMarker(0.5,0.6,()->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.DOWN.val);
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                })
                .addTemporalMarker(0.5,0.8,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
                })
                .splineToConstantHeading(new Vector2d(17.2,1),Math.toRadians(0))
                .build();
        spikeMarks[0] = bot.drive.trajectorySequenceBuilder(startPhase[0].end())
                //spike mark 1
                .addTemporalMarker(0.2,0.1,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.PARTIAL_EXTEND);
                })
                .addTemporalMarker(0.4,0.05,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);
                })
                .addTemporalMarker(0.4,0.43,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                })
                .addTemporalMarker(0.52,0.45,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.MIDDLE);

                })
                .addTemporalMarker(0.89,0.1,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
                })
                .lineToLinearHeading(new Pose2d(21.8, -30.5, Math.toRadians(-25)))
                .splineToLinearHeading(new Pose2d(19, -41, Math.toRadians(-150)),Math.toRadians(0))
                .build();
        spikeMarks[1] = bot.drive.trajectorySequenceBuilder(spikeMarks[0].end())
  //              leave spike mark 1 + spike mark 2
                .addTemporalMarker(0.1,0.1,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.MIDDLE);
                })
                .addTemporalMarker(0.2,0.28,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);
                })
                .addTemporalMarker(0.2,0.49,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                })
                .addTemporalMarker(0.2,0.54,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
                })
                .addTemporalMarker(0.2,0.65,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);

                })
                .addTemporalMarker(0.33,0.5,()->{
                    bot.claw.clawHRotate(Claw.HORIZONTAL_STATES.PARALEL);
                })
                .addTemporalMarker(0.5,0.3,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                })
                .addTemporalMarker(0.7,0.1,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.MIDDLE);
                })
                .addTemporalMarker(0.8,0.1,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
                })
                .addTemporalMarker(0.89,0.4,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                })

                .lineToLinearHeading(new Pose2d(26, -43.7, Math.toRadians(-10)))
                .splineToLinearHeading(new Pose2d(19.6, -45, Math.toRadians(-150)),Math.toRadians(0))
                .build();
        spikeMarks[2] = bot.drive.trajectorySequenceBuilder(spikeMarks[1].end())
                .addTemporalMarker(0.6,0.2,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.PARTIAL_EXTEND);
                })
                .addTemporalMarker(0.68,0.3,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);
                })
                .addTemporalMarker(0.68,0.35,()->{
                    bot.claw.clawHRotate(Claw.HORIZONTAL_STATES.PERPENDICULAR);
                })
                .addTemporalMarker(0.73,0.85,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                })

                .lineToLinearHeading(new Pose2d(39.4, -41.5, Math.toRadians(-90)))

                .build();

        spikeMarks[3] = bot.drive.trajectorySequenceBuilder(spikeMarks[2].end())

                .addTemporalMarker(0,0.3,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                    bot.claw.clawHRotate(Claw.HORIZONTAL_STATES.PARALEL);
                })

                .addTemporalMarker(0.4,0.1, () -> {
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.MIDDLE);
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.LOWMID.val);
                })
                .addTemporalMarker(0.6,0.1,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.LOWMID);
                })
                .addTemporalMarker(0.5,0.25,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
                })
                .addTemporalMarker(0.8,0.3,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                })
                .addTemporalMarker(0.9,0,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.PARTIAL_EXTEND);
                })

                .splineToLinearHeading(new Pose2d(14.4, -31.5, Math.toRadians(-178)),Math.toRadians(20))
                .build();


            place[0] = bot.drive.trajectorySequenceBuilder(spikeMarks[3].end())
                    //place first nonpreload specimen on fence

                    .addTemporalMarker(0, 0.05, () ->
                    {
                        bot.linkage.linkageMove(Linkage.EXTEND_STATES.PARTIAL_EXTEND);

                    }
                    )
                    .addTemporalMarker(0 , 0.2 , ()->
                    {
                        bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                    })
                    .addTemporalMarker(0,0.4,()->{
                        bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
                    })

                    .addTemporalMarker(1.2,()->{
                        bot.lifter.setTarget(Lifteer.LIFTER_STATES.MIDDLE.val);
                    })
                    .addTemporalMarker(2,()->{
                        bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                        bot.linkage.linkageMove(Linkage.EXTEND_STATES.PARTIAL_EXTEND);
                    })
                    .addTemporalMarker(2.5,()->{
                        bot.lifter.setTarget(Lifteer.LIFTER_STATES.SPECIMEN.val);
                    })
                    .addTemporalMarker(3.2,()->{
                        bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);

                    })
                    .addTemporalMarker(3.25,()->{
                        bot.linkage.linkageMove(Linkage.EXTEND_STATES.MIDDLE);
                    })
                    .addTemporalMarker(3.35,()->{

                        bot.lifter.setTarget(Lifteer.LIFTER_STATES.LOWMID.val);
                    })

                    .addTemporalMarker(0.85,0.2,()->{
                        bot.linkage.linkageMove(Linkage.EXTEND_STATES.PARTIAL_EXTEND);
                    })

                    .lineToLinearHeading(new Pose2d(13.5,-32,Math.toRadians(180)))
                    .waitSeconds(0.1)

                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(15.15,0,  Math.toRadians(10)),Math.toRadians(0))

                    .build();

        place[1] = bot.drive.trajectorySequenceBuilder(place[0].end())
                //place first nonpreload specimen on fence

                .addTemporalMarker(0.42, 0, () ->
                        {
                            bot.linkage.linkageMove(Linkage.EXTEND_STATES.PARTIAL_EXTEND);


                        }
                )

                .addTemporalMarker(0.42, 0.2 , () ->
                {

                    bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                })
                .addTemporalMarker(2.5,()->{


                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.MIDDLE.val);
                })
                .addTemporalMarker(3,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.PARTIAL_EXTEND);
                })
                .addTemporalMarker(4,()->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.SPECIMEN.val);
                })
                .addTemporalMarker(4.4,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.MIDDLE);
                })
                .addTemporalMarker(5.25,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
                })
                .addTemporalMarker(5,()->{

                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.LOWMID.val);
                })
                .lineToLinearHeading(new Pose2d(13.5, -30,Math.toRadians(180)))
                .waitSeconds(0.1)

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(16.2,-2,Math.toRadians(1)),Math.toRadians(50))

                .build();

        place[2] = bot.drive.trajectorySequenceBuilder(place[1].end())
                //place first nonpreload specimen on fence

                .addTemporalMarker(0.42, 0, () ->
                        {
                            bot.linkage.linkageMove(Linkage.EXTEND_STATES.PARTIAL_EXTEND);

                        }
                )
                .addTemporalMarker(0.42, 0.2 , () ->
                {

                    bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                })
                .addTemporalMarker(2.5,()->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.MIDDLE.val);
                })
                .addTemporalMarker(3,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.PARTIAL_EXTEND);
                })
                .addTemporalMarker(4.3,()->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.SPECIMEN.val);
                })
                .addTemporalMarker(4.4,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.MIDDLE);
                })
                .addTemporalMarker(5.4,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
                })
                .addTemporalMarker(5.4,()->{

                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.LOWMID.val);
                })
                .lineToLinearHeading(new Pose2d(12.5,-30,Math.toRadians(170)))
                .waitSeconds(0.1)

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(13.4,-2,Math.toRadians(1)),Math.toRadians(50))

                .build();




        park[0] = bot.drive.trajectorySequenceBuilder(place[2].end())
                .lineToLinearHeading(new Pose2d(14, -31.5, Math.toRadians(-172)))
                .addDisplacementMarker(()->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.DOWN.val);
                })
                .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        bot.Init(hardwareMap, telemetry);
        bot.lifter.set_motorsResetable(false);

        telemetry.addLine("non comience el espectaculo");
        telemetry.update();


        buildTrajectories();
        telemetry.addLine("que comience el espectaculo");
        telemetry.update();

        waitForStart();

        if(isStopRequested()) return;

        bot.drive.setPoseEstimate(new Pose2d(0.1,0,Math.toRadians(0)));
        bot.drive.followTrajectorySequenceAsync(startPhase[0]);

        while ((bot.drive.isBusy() && !isStopRequested())) {
            bot.drive.update();
            bot.lifter.update();
            telemetry.addData("er", bot.drive.getLastError());
            telemetry.update();
        }

        bot.drive.followTrajectorySequenceAsync(spikeMarks[0]);

        while ((bot.drive.isBusy() && !isStopRequested())) {
            bot.drive.update();
            telemetry.addData("er", bot.drive.getLastError());
            bot.lifter.update();
            telemetry.update();
        }
        bot.drive.followTrajectorySequenceAsync(spikeMarks[1]);

        while ((bot.drive.isBusy() && !isStopRequested())) {
            bot.drive.update();
            telemetry.addData("er", bot.drive.getLastError());
            bot.lifter.update();
            telemetry.update();
        }
//      sleep(500);

        bot.drive.followTrajectorySequenceAsync(spikeMarks[2]);

        while ((bot.drive.isBusy() && !isStopRequested())) {
            bot.drive.update();
            telemetry.addData("er", bot.drive.getLastError());
            bot.lifter.update();
            telemetry.update();
        }

        bot.drive.followTrajectorySequenceAsync(spikeMarks[3]);


        while ((bot.drive.isBusy() && !isStopRequested())) {
            bot.drive.update();
            telemetry.addData("er", bot.drive.getLastError());
            bot.lifter.update();
            telemetry.update();
        }

        bot.drive.followTrajectorySequenceAsync(place[0]);
        while ((bot.drive.isBusy() && !isStopRequested())) {
            bot.drive.update();
            telemetry.addData("er", bot.drive.getLastError());
            bot.lifter.update();
            telemetry.update();
        }


         bot.drive.followTrajectorySequenceAsync(place[1]);
        while ((bot.drive.isBusy() && !isStopRequested())) {
            bot.drive.update();
            telemetry.addData("er", bot.drive.getLastError());
            bot.lifter.update();
            telemetry.update();
        }

//
//        bot.drive.followTrajectorySequenceAsync(place[2]);
//        while ((bot.drive.isBusy() && !isStopRequested())) {
//            bot.drive.update();
//            telemetry.addData("er", bot.drive.getLastError());
//            bot.lifter.update();
//            telemetry.update();
//        }




        telemetry.addLine("c'est fini");
        telemetry.addData("timp: ",getRuntime());
        telemetry.update();



    };
}
