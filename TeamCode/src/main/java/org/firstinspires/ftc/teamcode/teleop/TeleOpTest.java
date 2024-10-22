package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Robot;
@TeleOp
public class TeleOpTest extends LinearOpMode {

    Robot bot = new Robot();
    public double weight = 1;
    public boolean closed = true ;

    //C ctrl1, ctrl2;
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            //drive

            bot.drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x * 1.4,
                    -gamepad1.right_stick_x
            ).times(weight));
            //slow
            if (gamepad1.right_trigger > 0.5) {
                weight = 0.5;
            } else {
                weight = 1;
            }

            //claw

            if(gamepad1.cross == true){
                if(closed == true)
                { bot.claw.ClawOpen();
                  closed = false;
                }
                else {
                    bot.claw.ClawClose();
                    closed = true;
                }

            }


            //lifter
            if(gamepad1.left_bumper == true && gamepad1.right_bumper == false ){
                bot.lifter.LifterClose();
            }
            else if(gamepad1.right_bumper == true && gamepad1.left_bumper == false ){
                bot.lifter.LifterExtend();
            }


            //rotation
            if(gamepad1.dpad_up==true){
                bot.clawRotation.rotate(0);
            }
            else if(gamepad1.dpad_down==true){
                bot.clawRotation.rotate(1);

            }
            else if(gamepad1.dpad_right==true){
                bot.clawRotation.rotate(0.6);
            }
        }
    }

}