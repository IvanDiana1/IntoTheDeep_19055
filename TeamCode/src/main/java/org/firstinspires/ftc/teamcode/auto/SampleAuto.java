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
public class SampleAuto extends LinearOpMode {
    Robot bot = new Robot();
    public static TrajectorySequence startPhase[] = new TrajectorySequence[1], toSpikeMark[] = new TrajectorySequence[1];

    public void buildTrajectories(){
        startPhase[0] = bot.drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .addSpatialMarker(new Vector2d(0,0), () ->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.MIDDLE.val);
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
                })
                .addTemporalMarker(2400, () ->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);
                    sleep(300);
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.DOWN.val);
                    bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
                    sleep(100);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
                })
                .lineTo( new Vector2d( 16 , -4))
                .build();
        toSpikeMark[0] = bot.drive.trajectorySequenceBuilder(startPhase[0].end())
                .addSpatialMarker(new Vector2d(16,0), () ->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);
                    bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);

                })
                .setReversed(true)
                .splineToLinearHeading( new Pose2d(  48 , - 33 , Math.toRadians(-90)) , Math.toRadians(-30))

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
        bot.drive.followTrajectorySequenceAsync(toSpikeMark[0]);

        while ((bot.drive.isBusy() && !isStopRequested())) {
            bot.drive.update();
            bot.lifter.update();
            telemetry.addData("er", bot.drive.getLastError());
            telemetry.update();
        }

        telemetry.addLine("c'est fini");
        telemetry.addData("timp: ",getRuntime());
        telemetry.update();
    }
}
