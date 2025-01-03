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
public class SpecimenAutoClawTest extends LinearOpMode {
    Robot bot = new Robot();
    private TrajectorySequence lastraj;
    private static final double dist_between_specimens = 2;
    public static TrajectorySequence startPhase[] = new TrajectorySequence[1], spikeMarks[] = new TrajectorySequence[4] , place[] = new TrajectorySequence[3] , collect[] = new TrajectorySequence[1];
    public static Pose2d marksPosFence[] = new  Pose2d[3] ;
    public static Pose2d marksPosWall[] = new Pose2d[3];

    public void collectFromWall(){
        bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
        sleep(250);
        bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
        sleep(150);
        bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
    }
    public void placeSpecimen(int casee){

        bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);
        sleep(280);
        if (casee<2)
            bot.lifter.setTarget(Lifteer.LIFTER_STATES.LOWMID.val);
        else
            bot.lifter.setTarget(Lifteer.LIFTER_STATES.DOWN.val);


        bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
        if(casee<2||casee==3){
            sleep(80);
            bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
        }
    }

    public void buildTrajectories(double voltage) {
        marksPosFence[0] = new Pose2d(15,2,  Math.toRadians(5));
        marksPosFence[1] = new Pose2d(15.8,4,Math.toRadians(2));
        marksPosFence[2] = new Pose2d(16.9,0,Math.toRadians(-1));

        marksPosWall[0] = new Pose2d(13.7 ,-31.5,Math.toRadians(-172));
        marksPosWall[1] = new Pose2d(13.25,-32.1,Math.toRadians(-177));
        marksPosWall[2] = new Pose2d(11.8,-32.8,Math.toRadians(-150));
        startPhase[0] = bot.drive.trajectorySequenceBuilder(new Pose2d(-0.2, 0, Math.toRadians(0)))
                .addSpatialMarker(new Vector2d(0, 0), () -> {
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.MIDDLE.val);
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
                })
                .addTemporalMarker(2400, () -> {
                    placeSpecimen(3);
                })
                .forward(16)
                .build();
        spikeMarks[0] = bot.drive.trajectorySequenceBuilder(startPhase[0].end())
                //spike mark 1
                .addSpatialMarker(new Vector2d(23, -30), () -> {
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);
                    sleep(300);
                    bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                    sleep(200);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                })
                .lineToLinearHeading(new Pose2d(23, -31, Math.toRadians(-30)))
                .lineToLinearHeading(new Pose2d(19, -41, Math.toRadians(-155)))
                .build();
        spikeMarks[1] = bot.drive.trajectorySequenceBuilder(spikeMarks[0].end())
                //leave spike mark 1 + spike mark 2
                .addSpatialMarker(new Vector2d(24.13, -41.9), () -> {
                    sleep(10);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);
                    sleep(300);
                    bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                    sleep(50);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                    sleep(150);
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.MIDDLE);
                })
                .lineToLinearHeading(new Pose2d(24., -42.15, Math.toRadians(-20)))
                .lineToLinearHeading(new Pose2d(19.6, -48, Math.toRadians(-155)))
                .build();
        spikeMarks[2] = bot.drive.trajectorySequenceBuilder(spikeMarks[1].end())

                .lineToLinearHeading(new Pose2d(25.23, -52., Math.toRadians(-25)))

                .build();

        spikeMarks[3] = bot.drive.trajectorySequenceBuilder(spikeMarks[2].end())
                .addTemporalMarker(600, () -> {
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.LOWMID.val);
                    sleep(150);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.LOWMID);
                })
                .lineToLinearHeading(new Pose2d(14.2, -31.5, Math.toRadians(-172)))
                .build();

        lastraj = spikeMarks[3];

        for (int i=0;i<=2;i++) {
            int finalI = i;

            place[i] = bot.drive.trajectorySequenceBuilder(lastraj.end())
                    .lineToLinearHeading(marksPosFence[i])

                    .addTemporalMarker(0.07, 0.08, () ->
                            {
                                bot.lifter.setTarget(Lifteer.LIFTER_STATES.MIDDLE.val);
                                bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
                            }
                    )
                    .lineToLinearHeading(marksPosWall[i])

                    .addTemporalMarker(0.4, 0.1, () -> {

                        placeSpecimen(finalI);
                    })


                    .build();
            lastraj = place[i];
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        bot.Init(hardwareMap, telemetry);
        buildTrajectories(bot.voltage_sensor.getVoltage());

        waitForStart();

        if(isStopRequested()) return;

        bot.drive.setPoseEstimate(new Pose2d(-0.2,0,Math.toRadians(0)));
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
        sleep(400);
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
        bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);

        for (int i=0;i<=2;i++) {
            collectFromWall();
            bot.drive.followTrajectorySequenceAsync(place[i]);
            while ((bot.drive.isBusy() && !isStopRequested())) {
                bot.drive.update();
                telemetry.addData("er", bot.drive.getLastError());
                bot.lifter.update();
                telemetry.update();
            }
        }
        telemetry.addLine("c'est fini");
        telemetry.addData("timp: ",getRuntime());
        telemetry.update();
    };
}