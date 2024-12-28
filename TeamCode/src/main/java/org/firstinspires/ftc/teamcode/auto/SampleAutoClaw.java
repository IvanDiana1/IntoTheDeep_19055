package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Claw;
import org.firstinspires.ftc.teamcode.util.Lifteer;
import org.firstinspires.ftc.teamcode.util.Linkage;
import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous
public class SampleAutoClaw extends LinearOpMode {
    Robot bot = new Robot();
    public static TrajectorySequence startPhase[] = new TrajectorySequence[1], spikeMarks[] = new TrajectorySequence[4] , place[] = new TrajectorySequence[2];

    public void buildTrajectories() {
        startPhase[0] = bot.drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .addSpatialMarker(new Vector2d(0, 0), () -> {
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.MIDDLE.val);
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
                })
                .addTemporalMarker(2400, () -> {
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);
                    sleep(300);
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.DOWN.val);
                    bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
                    sleep(100);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
                })
                .forward(16)
                .build();
        spikeMarks[0] = bot.drive.trajectorySequenceBuilder(startPhase[0].end())
                //spike mark 1
                .addSpatialMarker(new Vector2d(23, -30), () -> {
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);
                    sleep(300);
                    bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                    sleep(300);
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

                .lineToLinearHeading(new Pose2d(25.5, -52, Math.toRadians(-25)))

                .build();

        spikeMarks[3] = bot.drive.trajectorySequenceBuilder(spikeMarks[2].end())
                .addTemporalMarker(600, ()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
                    bot.lifter.setTarget(400);
                    sleep(150);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.LOWMID);
                })
                .lineToLinearHeading(new Pose2d (14.2 , - 31.5 , Math.toRadians(-172)))
                .build();

        place[0] = bot.drive.trajectorySequenceBuilder(spikeMarks[3].end())
                .lineToLinearHeading( new Pose2d( 17.4 , -3 , Math.toRadians(5)))
                .addSpatialMarker(new Vector2d( 16, -18), () ->
                        {   bot.lifter.setTarget(Lifteer.LIFTER_STATES.MIDDLE.val);
                            bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
                        })

                .addTemporalMarker(2400, () -> {
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);
                    sleep(300);
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.DOWN.val);
                    bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
                    sleep(100);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
                })
                .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        bot.Init(hardwareMap, telemetry);
        waitForStart();
        buildTrajectories();
        if(isStopRequested()) return;
        bot.drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));
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


        bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
        sleep(250);
        bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
        sleep(150);
        bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);

        bot.drive.followTrajectorySequenceAsync(place[0]);


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
