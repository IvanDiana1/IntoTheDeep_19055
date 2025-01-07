package org.firstinspires.ftc.teamcode.auto;


import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive3;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Claw;
import org.firstinspires.ftc.teamcode.util.Lifteer;
import org.firstinspires.ftc.teamcode.util.Linkage;
import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous
public class SpecimenAutoRemake extends LinearOpMode {
    Robot bot = new Robot();
    public static TrajectorySequence startPhase[] = new TrajectorySequence[1], spikeMarks[] = new TrajectorySequence[3] , place[] = new TrajectorySequence[3];

    public void collectFromWall(){
        bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
        sleep(100);
        bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
        sleep(400);
        bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
        sleep(200);
        bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
        sleep(100);
        bot.linkage.linkageMove(Linkage.EXTEND_STATES.MIDDLE);
    }


    public void buildTrajectories(){
        startPhase[0] = bot.drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .addSpatialMarker(new Vector2d(0, 0), () -> {
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.MIDDLE.val);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
                })
                .addTemporalMarker(1.3,()->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.SPECIMEN.val);
                })
                .addTemporalMarker(1.9,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.DOWN.val);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
                })
                .lineToLinearHeading(new Pose2d(16,7,Math.toRadians(0)),
                        SampleMecanumDrive3.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive3.getAccelerationConstraint(22.5))
                .build();

        spikeMarks[0] = bot.drive.trajectorySequenceBuilder(startPhase[0].end())
                //collect sample from first spike mark
                .addTemporalMarker(0.5,0,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);
                })
                .addTemporalMarker(0.6,0,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                })
                //raise sample from first spike mark
                .addTemporalMarker(0.6,0.35,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.MIDDLE);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                })
                //leave sample from first spike mark in observation zone
                .addTemporalMarker(1,0,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
                })
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(21.5, -40),Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(16, -50, Math.toRadians(-180)),Math.toRadians(0))
                .build();

        spikeMarks[1] = bot.drive.trajectorySequenceBuilder(spikeMarks[0].end())
                //collect sample from second spike mark
                .addTemporalMarker(0,0,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                })
                .addTemporalMarker(0.35,0.1,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);
                })
                .addTemporalMarker(0.35,0.3,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                })
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(21.3, -50, Math.toRadians(0)),Math.toRadians(0),
                        SampleMecanumDrive3.getVelocityConstraint(45,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive3.getAccelerationConstraint(25))

                //raise sample from second spike mark
                .addTemporalMarker(0.4,0.5,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                })
                .addTemporalMarker(0.4,0.6,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.MIDDLE);
                })

                //leave sample from second spike mark in observation zone
                //+raise lifter a bit to prepare for last spike mark
                .addTemporalMarker(0.9,0,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.LOWMID.val);
                })
                .splineToLinearHeading(new Pose2d(16, -50, Math.toRadians(-180)),Math.toRadians(0),
                        SampleMecanumDrive3.getVelocityConstraint(45,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive3.getAccelerationConstraint(20))

                .build();

        spikeMarks[2] = bot.drive.trajectorySequenceBuilder(spikeMarks[1].end())

                //put the linkage and claw into position to collect the last sample
                .addTemporalMarker(0.2,0,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.HIGHMID);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);
                    bot.claw.clawHRotate(Claw.HORIZONTAL_STATES.PERPENDICULAR);
                })

                //start lowering lifter
                .addTemporalMarker(0.35,0,()->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.DOWN.val);
                })

                //collect last sample from spikemark
                .addTemporalMarker(0.35,0.1,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                })

                //raise last sample from spikemark
                .addTemporalMarker(0.45,0.3,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                    bot.claw.clawHRotate(Claw.HORIZONTAL_STATES.PARALEL);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                })
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(36, -46, Math.toRadians(-90)),Math.toRadians(0))

                //leave sample in observation area
                .addTemporalMarker(0.6,0.2,()->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.LOWMID.val);
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
                    bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
                })
                .addTemporalMarker(0.6,0.4,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                })
                .addTemporalMarker(0.8,0,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
                })

                .splineToLinearHeading(new Pose2d(20.3, -35, Math.toRadians(-180)),Math.toRadians(0))

                //collect specimen from wall
                .addTemporalMarker(1,0,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                })
                //velocity constraint to not bump the wall too hard
                .setReversed(false)
                .lineToConstantHeading(new Vector2d(13,-35))

                .build();

        place[0] = bot.drive.trajectorySequenceBuilder(spikeMarks[2].end())

                //collect specimen from wall
                .addTemporalMarker(0,0,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
                })
                //place first nonpreload specimen on fence
                .addTemporalMarker(0.2,0.1,()->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.MIDDLE.val);
                })
                .addTemporalMarker(2,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.SPECIMEN.val);
                })
                .addTemporalMarker(2.8,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.DOWN.val);
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                })
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(20,-4,Math.toRadians(0)))

                //prepare for collecting second specimen from wall
                .addTemporalMarker(0.9,0,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
                })

                .splineToLinearHeading(new Pose2d(20.5, -35, Math.toRadians(-210)),Math.toRadians(0),
                        SampleMecanumDrive3.getVelocityConstraint(45,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive3.getAccelerationConstraint(25))

                //collect specimen from wall
                .addTemporalMarker(1,0,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                })
                //velocity constraint to not bump the wall too hard
                .setReversed(false)
                .lineToConstantHeading(new Vector2d(13,-35))

                .build();

        place[1] = bot.drive.trajectorySequenceBuilder(place[0].end())

                //place second nonpreload specimen on fence
                .addTemporalMarker(0,0,()->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.MIDDLE.val);
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
                })
                .addTemporalMarker(0.2,0,()->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.SPECIMEN.val);
                })
                .addTemporalMarker(0.2,0.1,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.LOWMID.val);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                })
                .splineTo(new Vector2d(20,1),Math.toRadians(0))

                //prepare for collecting third specimen from wall
                .addTemporalMarker(0.7,0,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
                })
                .splineTo(new Vector2d(15,-35),Math.toRadians(-180))


                //collect specimen from wall
                .addTemporalMarker(1,0,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                })
                //velocity constraint to not bump the wall too hard
                .setVelConstraint(new MecanumVelocityConstraint(25 , 9.65))
                .setAccelConstraint(new ProfileAccelerationConstraint(15))
                .lineToConstantHeading(new Vector2d(11,-35))
                .build();

        place[2] = bot.drive.trajectorySequenceBuilder(place[1].end())

                //place third nonpreload specimen on fence
                .addTemporalMarker(0,0,()->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.MIDDLE.val);
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
                })
                .addTemporalMarker(0.2,0,()->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.SPECIMEN.val);
                })
                .addTemporalMarker(0.2,0.1,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.LOWMID.val);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                })
                .splineTo(new Vector2d(20,-3),Math.toRadians(0))

                //park
                .addTemporalMarker(0.7,0,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
                })
                .splineTo(new Vector2d(11,-35),Math.toRadians(-180))
                .build();

    }

    @Override
    public void runOpMode() throws InterruptedException {
        bot.Init(hardwareMap, telemetry);
        bot.lifter.set_motorsResetable(false);


        buildTrajectories();

        waitForStart();

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

        bot.drive.followTrajectorySequenceAsync(spikeMarks[1]);

        while ((bot.drive.isBusy() && !isStopRequested())) {
            bot.drive.update();
            telemetry.addData("er", bot.drive.getLastError());
            bot.lifter.update();
            telemetry.update();
        }

        bot.drive.followTrajectorySequenceAsync(spikeMarks[2]);

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

        sleep(5000);

        bot.drive.followTrajectorySequenceAsync(place[1]);

        while ((bot.drive.isBusy() && !isStopRequested())) {
            bot.drive.update();
            telemetry.addData("er", bot.drive.getLastError());
            bot.lifter.update();
            telemetry.update();
        }

        sleep(5000);

        bot.drive.followTrajectorySequenceAsync(place[2]);

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
