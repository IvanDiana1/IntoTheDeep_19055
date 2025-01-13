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
    private TrajectorySequence lastraj;
    private static final double dist_between_specimens = 2;
    public static TrajectorySequence park[] = new TrajectorySequence[2] ,startPhase[] = new TrajectorySequence[1], spikeMarks[] = new TrajectorySequence[4] , place[] = new TrajectorySequence[3] , collect[] = new TrajectorySequence[1];
    public static Pose2d marksPosFence[] = new  Pose2d[3] ;
    public static Pose2d marksPosWall[] = new Pose2d[3];

    public void collectFromWall(){
        bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
        sleep(300);
        bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
        sleep(300);
        bot.linkage.linkageMove(Linkage.EXTEND_STATES.MIDDLE);
    }

    public void buildTrajectories(double voltage) {
        marksPosFence[0] = new Pose2d(18.5,-4,  Math.toRadians(-10));
        marksPosFence[1] = new Pose2d(19,-2,Math.toRadians(-10));
        marksPosFence[2] = new Pose2d(19,-7,Math.toRadians(-10));

        marksPosWall[0] = new Pose2d(13,-32,Math.toRadians(180));
        marksPosWall[1] = new Pose2d(13,-32,Math.toRadians(180));
        marksPosWall[2] = new Pose2d(13,-32,Math.toRadians(180));
        startPhase[0] = bot.drive.trajectorySequenceBuilder(new Pose2d(-0.2, 0, Math.toRadians(0)))
                .addSpatialMarker(new Vector2d(0, 0), () -> {
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.MIDDLE.val);
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
                })
                .addTemporalMarker(2400, () -> {
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);

                    //placeSpecimen(3);
                })
                .addTemporalMarker(2750,()->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.DOWN.val);
                    bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);

                })
                .addTemporalMarker(2830,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
                })
                .splineToConstantHeading(new Vector2d(16,1),Math.toRadians(0))
                .build();
        spikeMarks[0] = bot.drive.trajectorySequenceBuilder(startPhase[0].end())
                //spike mark 1
                .addTemporalMarker(0,0.1,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);
                })
                .addTemporalMarker(0,0.350,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                })
                .addTemporalMarker(0,0.550,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);

                })
                .lineToLinearHeading(new Pose2d(23, -31, Math.toRadians(-30)))
                .splineToLinearHeading(new Pose2d(19, -41, Math.toRadians(-120)),Math.toRadians(0))
                .build();
        spikeMarks[1] = bot.drive.trajectorySequenceBuilder(spikeMarks[0].end())
                //leave spike mark 1 + spike mark 2
                .addTemporalMarker(20,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);
                })
                .addTemporalMarker(320,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                })
                .addTemporalMarker(370,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                })
                .addTemporalMarker(520,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.MIDDLE);
                })

                .lineToLinearHeading(new Pose2d(24, -42.15, Math.toRadians(-20)))
                .splineToLinearHeading(new Pose2d(19.6, -48, Math.toRadians(-120)),Math.toRadians(0))
                .build();
        spikeMarks[2] = bot.drive.trajectorySequenceBuilder(spikeMarks[1].end())

                .lineToLinearHeading(new Pose2d(35, -46, Math.toRadians(-90)))

                .build();

        spikeMarks[3] = bot.drive.trajectorySequenceBuilder(spikeMarks[2].end())
                .addTemporalMarker(600, () -> {
                            bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
                            bot.lifter.setTarget(Lifteer.LIFTER_STATES.LOWMID.val);
                        })
                .addTemporalMarker(650,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
                })
                .addTemporalMarker(800,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                })

                .splineToLinearHeading(new Pose2d(14, -31.5, Math.toRadians(-178)),Math.toRadians(20))
                .build();

        lastraj = spikeMarks[3];


        for (int i=0;i<=2;i++) {

            place[i] = bot.drive.trajectorySequenceBuilder(lastraj.end())
                    //place first nonpreload specimen on fence

                    .addTemporalMarker(0, 0, () ->
                            {
                                bot.lifter.setTarget(Lifteer.LIFTER_STATES.MIDDLE.val);
                                bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
                            }
                    )
                    .addTemporalMarker(2,()->{
                        bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                    })
                    .addTemporalMarker(2.5,()->{
                        bot.lifter.setTarget(Lifteer.LIFTER_STATES.SPECIMEN.val);
                    })
                    .addTemporalMarker(2.9,()->{
                        bot.linkage.linkageMove(Linkage.EXTEND_STATES.MIDDLE);
                    })
                    .addTemporalMarker(3.2,()->{
                        bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
                    })
                    .addTemporalMarker(3.3,()->{

                        bot.lifter.setTarget(Lifteer.LIFTER_STATES.LOWMID.val);
                    })
                    .lineToLinearHeading(marksPosWall[i])
                    .waitSeconds(0.1)

                    .setReversed(true)
                    .splineToLinearHeading(marksPosFence[i],Math.toRadians(50))
                    .waitSeconds(0.1)

                    .build();
            lastraj = place[i];
        }

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

        buildTrajectories(bot.voltage_sensor.getVoltage());
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
        sleep(300);
        bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
        bot.drive.followTrajectorySequenceAsync(spikeMarks[1]);

        while ((bot.drive.isBusy() && !isStopRequested())) {
            bot.drive.update();
            telemetry.addData("er", bot.drive.getLastError());
            bot.lifter.update();
            telemetry.update();
        }
//      sleep(500);
        bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
        bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
        sleep(300);
        bot.drive.followTrajectorySequenceAsync(spikeMarks[2]);

        while ((bot.drive.isBusy() && !isStopRequested())) {
            bot.drive.update();
            telemetry.addData("er", bot.drive.getLastError());
            bot.lifter.update();
            telemetry.update();
        }

        bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
        bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
        sleep(400);
        bot.claw.clawVRotate(Claw.VERTICAL_STATES.LOWMID);
        sleep(200);
        bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
        sleep(100);
        bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
        bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);


        bot.drive.followTrajectorySequenceAsync(spikeMarks[3]);


        while ((bot.drive.isBusy() && !isStopRequested())) {
            bot.drive.update();
            telemetry.addData("er", bot.drive.getLastError());
            bot.lifter.update();
            telemetry.update();
        }

        sleep(200);
//
        for (int i=0;i<=1;i++) {
            collectFromWall();
            bot.drive.followTrajectorySequenceAsync(place[i]);
            while ((bot.drive.isBusy() && !isStopRequested())) {
                bot.drive.update();
                telemetry.addData("er", bot.drive.getLastError());
                bot.lifter.update();
                telemetry.update();
            }
        }
        bot.drive.followTrajectorySequenceAsync(park[0]);


        while ((bot.drive.isBusy() && !isStopRequested())) {
            bot.drive.update();
            telemetry.addData("er", bot.drive.getLastError());
            bot.lifter.update();
            telemetry.update();
        }



        telemetry.addLine("c'est fini");
        telemetry.addData("timp: ",getRuntime());
        telemetry.update();
    };
}
