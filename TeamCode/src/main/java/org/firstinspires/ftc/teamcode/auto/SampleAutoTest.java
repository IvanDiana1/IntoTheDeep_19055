package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.Claw;
import org.firstinspires.ftc.teamcode.util.Lifteer;
import org.firstinspires.ftc.teamcode.util.Linkage;
import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous
public class SampleAutoTest extends LinearOpMode {

    Robot bot = new Robot();

    public static TrajectorySequence startPhase[] = new TrajectorySequence[1], spikeMarks[] = new TrajectorySequence[4] , spikeMarksEnd[] = new TrajectorySequence[1], park[] = new TrajectorySequence[1];
    private static Thread uniqueThread;

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
 public void buildTrajectories(){
     startPhase[0] = bot.drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
             .addSpatialMarker(new Vector2d(0, 0), () -> {
                 bot.lifter.setTarget(Lifteer.LIFTER_STATES.MIDDLE.val);
                 bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
             })
             .addTemporalMarker(2400, () -> {
                 placeSpecimen(3);
             })
             .lineToConstantHeading( new Vector2d( 16, -5))
             .build();

     spikeMarks[0] = bot.drive.trajectorySequenceBuilder(startPhase[0].end())
             //spike mark 1
             .addSpatialMarker(new Vector2d(20, 22), () -> {
                 bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);
                 bot.claw.clawHRotate(Claw.HORIZONTAL_STATES.TILTED);
                 sleep(300);
                 bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                 sleep(200);
                 bot.claw.clawHRotate(Claw.HORIZONTAL_STATES.PARALEL);
                 bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                 bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
             })
             .lineToLinearHeading(new Pose2d(19.5, 23, Math.toRadians(30)))
             .lineToLinearHeading(new Pose2d(17, 40, Math.toRadians(160)))
             .addTemporalMarker(0.45 , 0.1 , ()->{
                 bot.claw.clawVRotate(Claw.VERTICAL_STATES.HIGHMID);
                 bot.lifter.setTarget(Lifteer.LIFTER_STATES.UP.val);

                     }
                     )
             .build();

     spikeMarks[1] = bot.drive.trajectorySequenceBuilder(spikeMarks[0].end())
             //spike mark 1
             .addSpatialMarker(new Vector2d(18, 41), () -> {

                 uniqueThread.interrupt();
                 sleep(100);
                 bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);
                 bot.linkage.linkageMove(Linkage.EXTEND_STATES.HIGHMID);
                 sleep(250);
                 bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                 sleep(250);
                 bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
                 sleep(100);
                 bot.linkage.linkageMove(Linkage.EXTEND_STATES.MIDDLE);
                 sleep(250);
                 bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                 bot.lifter.setTarget(Lifteer.LIFTER_STATES.UP.val);

             })
             .setAccelConstraint(new ProfileAccelerationConstraint(25))
             .setVelConstraint(new MecanumVelocityConstraint(25, 9.65))

             .lineToLinearHeading(new Pose2d(20, 42, Math.toRadians(0)))
             .setAccelConstraint(new ProfileAccelerationConstraint(3.5))
             .setVelConstraint(new MecanumVelocityConstraint(15, 9.65))
             .lineToLinearHeading(new Pose2d(17, 40, Math.toRadians(-160)))
             .lineToLinearHeading(new Pose2d(17.5 , 40.5, Math.toRadians(-210)))
             .build();

     spikeMarks[2] = bot.drive.trajectorySequenceBuilder(spikeMarks[1].end())
             //spike mark 1


             .lineToLinearHeading(new Pose2d(22, 41, Math.toRadians(35)))
             .build();
     spikeMarksEnd[0] = bot.drive.trajectorySequenceBuilder(spikeMarks[2].end())

             .lineToLinearHeading(new Pose2d(17, 36, Math.toRadians(-210)))
             .build();
     park[0] = bot.drive.trajectorySequenceBuilder(spikeMarksEnd[0].end())
             .lineToLinearHeading(new Pose2d(60,15,Math.toRadians(-90)))
             .build();
 }


    @Override
    public void runOpMode() throws InterruptedException {
        bot.Init(hardwareMap, telemetry);
        bot.lifter.setAutoPos(0);
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
        while (((bot.lifter.isBusy()||bot.drive.isBusy()) && !isStopRequested()) ) {
            bot.drive.update();
            telemetry.addData("er", bot.drive.getLastError());
            bot.lifter.update();
            telemetry.update();
        }
        bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
        sleep(400);
        bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
        sleep(200);
        bot.lifter.setTarget(Lifteer.LIFTER_STATES.DOWN.val);
        bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
        uniqueThread = new Thread(() -> {
            while (bot.lifter.isBusy() && !uniqueThread.isInterrupted()){
                bot.lifter.update();
            }
        });
        uniqueThread.start();
        sleep(500);
        bot.drive.followTrajectorySequenceAsync(spikeMarks[1]);
        while (((bot.lifter.isBusy()||bot.drive.isBusy()) && !isStopRequested())) {
            bot.drive.update();
            telemetry.addData("er", bot.drive.getLastError());
            bot.lifter.update();
            telemetry.update();
        }
        bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
        sleep(100);
        bot.claw.clawVRotate(Claw.VERTICAL_STATES.HIGHMID);
        sleep(300);
        bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
        sleep(200);
        bot.lifter.setTarget(Lifteer.LIFTER_STATES.DOWN.val);
        bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);


        uniqueThread = new Thread(() -> {
            while (bot.lifter.isBusy() && !uniqueThread.isInterrupted()){
                bot.lifter.update();
            }
        });
        uniqueThread.start();
        sleep(700);
        bot.drive.followTrajectorySequenceAsync(spikeMarks[2]);
        while (((bot.lifter.isBusy()||bot.drive.isBusy()) && !isStopRequested())) {
            bot.drive.update();
            telemetry.addData("er", bot.drive.getLastError());
            bot.lifter.update();
            telemetry.update();
        }


        uniqueThread.interrupt();
        sleep(300);
        bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);
        bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
        bot.claw.clawHRotate(Claw.HORIZONTAL_STATES.TILTED);
        sleep(300);
        bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
        sleep(200);
        bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
        sleep(250);
        bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
        bot.lifter.setTarget(Lifteer.LIFTER_STATES.UP.val);


        bot.drive.followTrajectorySequenceAsync(spikeMarksEnd[0]);
        while (((bot.lifter.isBusy()||bot.drive.isBusy()) && !isStopRequested())) {
            bot.drive.update();
            telemetry.addData("er", bot.drive.getLastError());
            bot.lifter.update();
            telemetry.update();
        }
        bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
        bot.claw.clawVRotate(Claw.VERTICAL_STATES.HIGHMID);
        sleep(400);
        bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);
        sleep(200);
        bot.lifter.setTarget(Lifteer.LIFTER_STATES.DOWN.val);
        bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);

        bot.lifter.update();
        bot.lifter.setAutoPos(bot.lifter.currentPos);
        sleep(500);


        telemetry.addLine("c'est fini");
        telemetry.addData("timp: ",getRuntime());
        telemetry.update();
    };

}
