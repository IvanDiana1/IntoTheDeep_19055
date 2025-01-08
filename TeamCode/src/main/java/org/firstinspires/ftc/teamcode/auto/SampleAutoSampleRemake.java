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
public class SampleAutoSampleRemake extends LinearOpMode {
    Robot bot = new Robot();

    public static TrajectorySequence startPhase[] = new TrajectorySequence[1],place [] = new TrajectorySequence[3] , parking[] = new TrajectorySequence[1];

public void buildTrajectories(){
    startPhase[0] = bot.drive.trajectorySequenceBuilder( new Pose2d(0 , 0 , Math.toRadians(0)))
            .addSpatialMarker( new Vector2d( 0 , 0) , () ->{
                bot.lifter.setTarget(Lifteer.LIFTER_STATES.UP.val);
            })
            .addTemporalMarker(0.5 ,0,  () ->
            {
                bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);

            })
            .addTemporalMarker(0.55, 0.3 , () ->
            {
                bot.claw.clawVRotate(Claw.VERTICAL_STATES.HIGHMID);
            })
            .addTemporalMarker(0.6,0 , () ->
            {
                bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
            })
            .lineToLinearHeading( new Pose2d( 10, 35, Math.toRadians(135)))

            .addDisplacementMarker(()->{
                bot.lifter.setTarget(Lifteer.LIFTER_STATES.DOWN.val);
                bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
            })

            .splineToLinearHeading(new Pose2d(19.8,23,Math.toRadians(30)),Math.toRadians(20))

            .build();

    place[0] = bot.drive.trajectorySequenceBuilder( startPhase[0].end())
            //vere ce plm e theme-ul asta ajutor
            .addSpatialMarker(new Vector2d(20,23),()->{
                bot.lifter.setTarget(Lifteer.LIFTER_STATES.UP.val);
           })
            .addTemporalMarker(0.8,0.1,()->{
                bot.linkage.linkageMove(Linkage.EXTEND_STATES.HIGHMID);
            })
            .addTemporalMarker(0.8,0.3,()->{
                bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
            })
            .addTemporalMarker(0.8,0.45,()->{
                bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
                bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                bot.lifter.setTarget(Lifteer.LIFTER_STATES.DOWN.val);
            })
            .lineToLinearHeading(new Pose2d(16, 40, Math.toRadians(160)))
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

        bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);
        bot.claw.clawHRotate(Claw.HORIZONTAL_STATES.TILTED);
        sleep(200);
        bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
        sleep(200);
        bot.claw.clawHRotate(Claw.HORIZONTAL_STATES.PARALEL);
        bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
        bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);

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

        bot.drive.followTrajectorySequenceAsync(place[2]);

        while ((bot.drive.isBusy() && !isStopRequested())) {
            bot.drive.update();
            telemetry.addData("er", bot.drive.getLastError());
            bot.lifter.update();
            telemetry.update();
        }


        bot.drive.followTrajectorySequenceAsync(parking[0]);

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
