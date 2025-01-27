package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Claw;
import org.firstinspires.ftc.teamcode.util.Lifteer;
import org.firstinspires.ftc.teamcode.util.Linkage;
import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous
public class Specimenx10Remakeuri extends LinearOpMode {

    static Robot bot = new Robot();
    public static TrajectorySequence park[] = new TrajectorySequence[2] ,startPhase[] = new TrajectorySequence[1], spikeMarks[] = new TrajectorySequence[4] , place[] = new TrajectorySequence[3] , collect[] = new TrajectorySequence[1];
    public static Pose2d marksPosFence[] = new  Pose2d[3] ;
    public static Pose2d marksPosWall[] = new Pose2d[3];
    public static double delta = 0.12;
    private static boolean builtStatically = true;
    private static String errorMessage=" ";


    public static void buildTrajectories(Robot bot) {
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
                .addTemporalMarker(0.2,0.3,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.PARTIAL_EXTEND);
                })
                .addTemporalMarker(0.5,0.05,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);

                })
                .addTemporalMarker(0.5, 0.4, ()-> {
                    bot.claw.clawHRotate(Claw.HORIZONTAL_STATES.LTILTED);
                })
                .addTemporalMarker(0.5,0.55,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                })
                .addTemporalMarker(0.5,0.63,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                    bot.claw.clawHRotate(Claw.HORIZONTAL_STATES.PARALEL);
                })
                .addTemporalMarker(0.63,0.52,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.MIDDLE);

                })
                .addTemporalMarker(0.63,0.67,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);

                })
                .lineToLinearHeading(new Pose2d(24.9, -29.5, Math.toRadians(-40)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(23, -33, Math.toRadians(-135)),Math.toRadians(90))
                .build();
        spikeMarks[1] = bot.drive.trajectorySequenceBuilder(spikeMarks[0].end())
                //              leave spike mark 1 + spike mark 2
                .addTemporalMarker(0,0.3,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.PARTIAL_EXTEND);
                })
                .addTemporalMarker(0.16,0.28,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);

                })
                .addTemporalMarker(0.16,0.3,()->{
                    bot.claw.clawHRotate(Claw.HORIZONTAL_STATES.LTILTED2);

                })
                .addTemporalMarker(0.16,0.47,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                })
                .addTemporalMarker(0.16,0.54,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                    bot.claw.clawHRotate(Claw.HORIZONTAL_STATES.PARALEL);
                })
                .addTemporalMarker(0.16,0.65,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.MIDDLE);

                })
                .addTemporalMarker(0.67,0.25,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
                })
                .addTemporalMarker(0.75, 0 , ()->
                {    bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);

                })
                .lineToLinearHeading(new Pose2d(27.6, -34.3, Math.toRadians(-40)))
                .splineToLinearHeading(new Pose2d(23, -37, Math.toRadians(-135)),Math.toRadians(0))
                .build();
        spikeMarks[2] = bot.drive.trajectorySequenceBuilder(new Pose2d(19.6, -45, Math.toRadians(-150)))

                .addTemporalMarker(0.4,0,()->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.LOWMID.val);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);
                    bot.claw.clawHRotate(Claw.HORIZONTAL_STATES.PERPENDICULAR);
                })
                .addTemporalMarker(0.6,0.2,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.HIGHMID);
                })
                //start lowering lifter
                .addTemporalMarker(0.63,0.45,()->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.DOWN.val);
                })
                //collect last sample from spikemark

                .addTemporalMarker(0.77,0.93,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.LOWMID);
                })
                //raise last sample from spikemark

                .lineToLinearHeading(new Pose2d(38.2, -44.2, Math.toRadians(-90)))
                .build();

        spikeMarks[3] = bot.drive.trajectorySequenceBuilder(spikeMarks[2].end())

                .addTemporalMarker(0,0.,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                })

                .addTemporalMarker(0,0.15,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                    bot.claw.clawHRotate(Claw.HORIZONTAL_STATES.PARALEL);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);

                })

                .addTemporalMarker(0.2, 0 , () -> {

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

                .splineToLinearHeading(new Pose2d(14.4, -31.8, Math.toRadians(-178)),Math.toRadians(20))
                .build();


        place[0] = bot.drive.trajectorySequenceBuilder(spikeMarks[3].end())
                //place first nonpreload specimen on fence

                .addTemporalMarker(0, 0.05, () ->
                {
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.PARTIAL_EXTEND);
                })
                .addTemporalMarker(0 , 0.2 , ()->
                {   bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                })
                .addTemporalMarker(0,0.4,()->{
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
                })

                .addTemporalMarker(1.15,()->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.MIDDLE.val);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);

                })
                .addTemporalMarker(1.8+delta,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
                })

                .addTemporalMarker(3.05+delta,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                })

                .addTemporalMarker(0.85,0.2,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.PARTIAL_EXTEND);
                })

                .lineToLinearHeading(new Pose2d(14.3,-32,Math.toRadians(180)))
                .waitSeconds(0.1)

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(16.3,0,  Math.toRadians(1)),Math.toRadians(0))
                .build();

        place[1] = bot.drive.trajectorySequenceBuilder(place[0].end())
                .addTemporalMarker(0,0.15,()->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.LOWMID.val);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                })
                //place first nonpreload specimen on fence
                .addTemporalMarker(0.25,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);

                })
                .addTemporalMarker(0.42, 0, () ->
                        {
                            bot.linkage.linkageMove(Linkage.EXTEND_STATES.PARTIAL_EXTEND);


                        }
                )

                .addTemporalMarker(0.42, 0.4 , () ->
                {
                    bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                })

                .addTemporalMarker(0.42, 0.46 , () ->
                {
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
                })
                .addTemporalMarker(2.65,()->{

                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);

                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.MIDDLE.val);
                })
                .addTemporalMarker(3,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
                })

                .addTemporalMarker(4.25 ,  () ->{
                            bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                        }
                )
                .addTemporalMarker(5.1,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
                })

                .addTemporalMarker(5,()->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.LOWMID.val);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                })

                .lineToLinearHeading(new Pose2d(12.9, -32.5,Math.toRadians(180)))
                .waitSeconds(0.1)

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(18.5,-2,Math.toRadians(1)),Math.toRadians(50))

                .build();

        place[2] = bot.drive.trajectorySequenceBuilder(place[1].end())
                //place first nonpreload specimen on fence
                .addTemporalMarker(0.3,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);

                })
                .addTemporalMarker(0.42, 0, () ->
                        {
                            bot.linkage.linkageMove(Linkage.EXTEND_STATES.PARTIAL_EXTEND);


                        }
                )

                .addTemporalMarker(0.42, 0.4 , () ->
                {
                    bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                })

                .addTemporalMarker(0.42, 0.46 , () ->
                {
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                })
                .addTemporalMarker(2.65,()->{

                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);

                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.MIDDLE.val);
                })
                .addTemporalMarker(3,()->{
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
                })

                .addTemporalMarker(4.25 ,  () ->{
                            bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                        }
                )
                .addTemporalMarker(5.23,()->{
                    bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
                })

                .addTemporalMarker(5,()->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.LOWMID.val);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                })

                .lineToLinearHeading(new Pose2d(11.95, -32.5,Math.toRadians(180)))
                .waitSeconds(0.1)

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(21.5,-2,Math.toRadians(1)),Math.toRadians(50))

                .build();



        park[0] = bot.drive.trajectorySequenceBuilder(place[2].end())
                .lineToLinearHeading(new Pose2d(5, -32.5, Math.toRadians(-90)))
                //.lineToLinearHeading(new Pose2d(12.25,-32.5,Math.toRadians(170)))

                .addTemporalMarker(0.5,0.1,()->{
                    bot.lifter.setTarget(Lifteer.LIFTER_STATES.DOWN.val);
                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
                    bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);

                })
                .build();
    }
    static {
        try{
            bot.FrozenInit();
            buildTrajectories(bot);
        }
        catch(Exception e){
            builtStatically = false;
            errorMessage = e.getMessage();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        bot.Init(hardwareMap,telemetry);
        bot.lifter.set_motorsResetable(false);
        if (!builtStatically) {

            telemetry.addLine("Trajectories COULDNT be built statically");
            telemetry.update();

            buildTrajectories(bot);

            telemetry.addLine("que comience el espectaculo");
            telemetry.addLine(errorMessage);
            telemetry.update();
        }
        else{
            telemetry.addLine("Trajectories SUCCESSFULLY built statically: ");
            telemetry.update();

        }

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
        sleep(500);

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


        bot.drive.followTrajectorySequenceAsync(place[2]);
        while ((bot.drive.isBusy() && !isStopRequested())) {
            bot.drive.update();
            telemetry.addData("er", bot.drive.getLastError());
            bot.lifter.update();
            telemetry.update();
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