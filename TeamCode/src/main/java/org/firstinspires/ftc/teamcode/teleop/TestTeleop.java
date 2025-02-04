package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Claw;
import org.firstinspires.ftc.teamcode.util.Lifteer;
import org.firstinspires.ftc.teamcode.util.Linkage;
import org.firstinspires.ftc.teamcode.util.Robot;


@TeleOp
public class TestTeleop extends LinearOpMode {

    public static Robot bot = new Robot();
    public Controller controller1, controller2;
    public double weight = 0.4;
    public Thread uniqueThread = new Thread();
    public static TrajectorySequence toObs[][] = new TrajectorySequence[3][3];
    public static TrajectorySequence toWall[][] = new TrajectorySequence[3][3];

    private static Drive_Mode driveMode = Drive_Mode.DRIVE_MODE;

    private static final Pose2d initialObservationPos = new Pose2d(13.5, -30.8, Math.toRadians(-180));
    private static final Pose2d initialWallPos = new Pose2d(17.7,0,  Math.toRadians(1));

    private static Pose2d wallPos[][];

    private static boolean builtStatically = true;
    private static String errorMessage=" ";


    int lifterInitial = 0;
    int dtlifter=0;

    static final double deltaX = 1.2;
    static final double deltaY = 1.5;

    static int currentTrajX = 0;
    static int currentTrajY = 0;

    public boolean isExtended = false;
    public boolean isUp = false;

    enum Drive_Mode {
        DRIVE_MODE,
        AUTO_MODE;
    }

    private static boolean atPickup = true;


    public static void buildTrajectories(){
        for (int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                toWall[i][j] = bot.drive.trajectorySequenceBuilder(initialObservationPos)
                        //place first nonpreload specimen on fence

                        .addTemporalMarker(0 , 0.05 , ()->
                        {   bot.claw.clawCatch(Claw.HOLD_STATES.HOLD);
                        })
                        .addTemporalMarker(0,0.25,()->{
                            bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
                        })

                        .addTemporalMarker(0.95,()->{
                            bot.lifter.setTarget(Lifteer.LIFTER_STATES.MIDDLE.val);
                            bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);

                        })
                        .addTemporalMarker(1.92,()->{
                            bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
                        })

                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(initialWallPos.getX()+deltaX*(i+(double)j/2.4), initialWallPos.getY()+deltaY*j, initialWallPos.getHeading()),Math.toRadians(0))
                        .build();

                toObs[i][j]= bot.drive.trajectorySequenceBuilder(toWall[i][j].end())
                        .addTemporalMarker(0,()->{
                            bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                        })
                        .addTemporalMarker(0,0.15,()->{
                            bot.lifter.setTarget(Lifteer.LIFTER_STATES.LOWMID.val);
                            bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                        })
                        //place first nonpreload specimen on fence
                        .addTemporalMarker(0.25,()->{
                            bot.claw.clawCatch(Claw.HOLD_STATES.RELEASE);

                        })
                        .addTemporalMarker(0.7, 0, () ->
                                {
                                    bot.linkage.linkageMove(Linkage.EXTEND_STATES.PARTIAL_EXTEND);
                                }
                        )


                        .lineToLinearHeading(initialObservationPos)
                        .build();

            }
        }
    }

    static {
        try{
            bot.FrozenInit();
            buildTrajectories();
        }
        catch(Exception e){
            builtStatically = false;
            errorMessage = e.getMessage();
        }
    }
    public void updateAll(){
        telemetry.addData("CURRENT TRAJ XXX",currentTrajX);
        telemetry.addData("CURRENT TRAJ YYY",currentTrajY);

        bot.lifter.telemetryData();
        bot.lifter.update();
        telemetry.update();
        controller1.update();
        controller2.update();

    }
    @Override
     public void runOpMode() throws InterruptedException{

        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        bot.Init(hardwareMap,telemetry);
        bot.lifter.setTarget(Lifteer.autoPos);

        if (!builtStatically) {

            telemetry.addLine("Trajectories COULDNT be built statically");
            telemetry.update();

            buildTrajectories();

            telemetry.addLine("que comience el espectaculo");
            telemetry.addLine(errorMessage);
            telemetry.update();
        }
        else{
            telemetry.addLine("Trajectories SUCCESSFULLY built statically: ");
            telemetry.update();

        }

        waitForStart();
        bot.lifter.reset_motors();
        bot.claw.clawVRotate(Claw.VERTICAL_STATES.DOWN);

        while(opModeIsActive() && !isStopRequested()){


            //drive
            if (driveMode == Drive_Mode.AUTO_MODE){
                updateAll();
                bot.drive.update();

                if (controller1.square.isPressed()) {
                    bot.drive.breakFollowing();
                    driveMode = Drive_Mode.DRIVE_MODE;
                }
                if(controller1.bumperLeft.isPressed()){
                    bot.claw.clawCatch();
                }
                if (!bot.drive.isBusy())
                    driveMode = Drive_Mode.DRIVE_MODE;
                continue;
            }

            bot.drive.setWeightedDrivePower( new Pose2d(
                    - gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x)
                    .times(1- weight*((double)max(1,bot.lifter.currentPos)/Lifteer.LIFTER_STATES.UP.val)));
            //slow drive
            if(controller1.bumperRight.isPressed()){
                weight = 0.7;
            } else
            if(controller1.bumperRight.isReleased()){
                weight = 0.4;
            }
// open-close claw
            if (controller1.bumperLeft.isPressed()) {
                bot.claw.clawCatch();
            }

            if (atPickup) {
                if (controller1.dpadDown.isPressed()) {
                    if (currentTrajX > 0)
                        currentTrajX--;
                }
                if (controller1.dpadUp.isPressed()) {
                    if (currentTrajX < 2)
                        currentTrajX++;
                }
                if (controller1.dpadLeft.isPressed()) {
                    if (currentTrajY < 2)
                        currentTrajY++;
                }
                if (controller1.dpadRight.isPressed()) {
                    if (currentTrajY > 0)
                        currentTrajY--;
                }
            }

            if(controller1.triangle.isPressed()){
                bot.claw.clawVRotate(Claw.VERTICAL_STATES.LOWMID);
            }
            if (controller1.cross.isPressed()){
                atPickup = true;
            }
            if (controller1.circle.isPressed()){
                if (atPickup) {
                    bot.drive.setPoseEstimate(toWall[currentTrajX][currentTrajY].start());
                    bot.drive.followTrajectorySequenceAsync(toWall[currentTrajX][currentTrajY]);
                    atPickup = false;
                }
                else {
                    bot.drive.setPoseEstimate(toObs[currentTrajX][currentTrajY].start());
                    bot.drive.followTrajectorySequenceAsync(toObs[currentTrajX][currentTrajY]);
                    atPickup = true;
                }
                driveMode = Drive_Mode.AUTO_MODE;
            }

            //close linkage
            if(controller2.triangle.isPressed()) {
                uniqueThread.interrupt();
                if (bot.lifter.Target == Lifteer.LIFTER_STATES.UP.val || bot.lifter.Target == Lifteer.LIFTER_STATES.MIDDLE.val ) {
                    bot.linkage.linkageMove();
                    isExtended = !isExtended;
                } else
                if( bot.lifter.Target == Lifteer.LIFTER_STATES.LOWMID.val){
                      if(isExtended) {

                          sleep(300);
                          if (Thread.currentThread().isInterrupted()) {
                              return;
                          }
                          bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                          isExtended = false;
                      }
                      else
                      {
                          bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                          if (Thread.currentThread().isInterrupted()) {
                              return;
                          }
                          bot.linkage.linkageMove(Linkage.EXTEND_STATES.PARTIAL_EXTEND);
                          isExtended = true;
                      }


                }


                else {
                    uniqueThread = new Thread(() -> {
                        if (isExtended) {
                            bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                            bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                            bot.claw.clawHRotate(Claw.HORIZONTAL_STATES.PARALEL);
                            sleep(300);
                            if (Thread.currentThread().isInterrupted()) {
                                return;
                            }

                            isExtended = false;

                            sleep(300);
                            if (Thread.currentThread().isInterrupted()) {
                                return;
                            }

                        } else {
                            bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
                            sleep(300);
                            if (Thread.currentThread().isInterrupted()) {
                                return;
                            }
                            bot.linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
                            isExtended = true;

                        }
                    }
                    );

                    uniqueThread.start();
                }
            }

//horizontal movement
            if(controller2.cross.isPressed()){

                if(isExtended){
                    bot.claw.clawHRotate();}



            }

            if(controller2.circle.isPressed()){
                if (bot.lifter.Target<Lifteer.LIFTER_STATES.LOWMID.val+260 && bot.lifter.Target>Lifteer.LIFTER_STATES.LOWMID.val-260 && bot.claw.vState == Claw.VERTICAL_STATES.MIDDLE)
                    bot.claw.clawVRotate(Claw.VERTICAL_STATES.UP);
                else
                    bot.claw.clawVRotate();
            }

            if (Math.abs(controller2.leftStickY)>0.2){
                if (controller2.leftStickY<0)
                    dtlifter+=4;
                else
                    dtlifter-=4;
                if (Math.abs(dtlifter)==4)
                    lifterInitial = bot.lifter.currentPos;
                bot.lifter.setTarget(lifterInitial+dtlifter);
            }
            else {
                lifterInitial = 0;
                dtlifter=0;
            }

            if(controller2.dpadUp.isPressed()){

             bot.lifter.setTarget(Lifteer.LIFTER_STATES.UP.val);
             isUp = true;
            }

            if(controller2.dpadDown.isPressed()){
                bot.lifter.setTarget(Lifteer.LIFTER_STATES.DOWN.val);
                sleep(400);
                bot.linkage.linkageMove(Linkage.EXTEND_STATES.CLOSE);
                isExtended = false;
                isUp = false;
            }

            if( controller2.dpadRight.isPressed()){
                bot.lifter.setTarget(Lifteer.LIFTER_STATES.MIDDAL.val);
                isUp = true;
            }
            if (controller2.dpadLeft.isPressed()){
                bot.lifter.setTarget(Lifteer.LIFTER_STATES.LOWMID.val);
                isUp = true;
            }
            if(controller2.bumperRight.isPressed() ){
                bot.claw.clawVRotate(Claw.VERTICAL_STATES.MIDDLE);
            }

            if(controller2.bumperLeft.isPressed()){
                bot.lifter.setTarget(Lifteer.LIFTER_STATES.AGATATED.val);
            }



            updateAll();


        }

     bot.lifter.setAutoPos(0);


    }

}
