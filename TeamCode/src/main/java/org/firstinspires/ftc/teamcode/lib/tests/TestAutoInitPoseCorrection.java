package org.firstinspires.ftc.teamcode.lib.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.AprilTagSmartDetection;
import org.firstinspires.ftc.teamcode.util.Camera;
import org.firstinspires.ftc.teamcode.util.Claw;
import org.firstinspires.ftc.teamcode.util.Lifteer;
import org.firstinspires.ftc.teamcode.util.Linkage;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.opencv.core.Mat;

@Autonomous
public class TestAutoInitPoseCorrection extends LinearOpMode {
    Robot bot;
    Camera cam;
    AprilTagSmartDetection tagpipeline = new AprilTagSmartDetection();
    AprilTagLibrary tagLibrary = AprilTagGameDatabase.getIntoTheDeepTagLibrary();

    // coords is expected to be at least 1 by 3 and CvType.CV_32F
    // theta must be normalized (degrees) and the mat should be in meters
    public Pose2d openCVMat2RRPose2d(Mat coords, double theta){
        double k = 1 / DistanceUnit.mPerInch;
        return new Pose2d(coords.get(0, 1)[0] * k, coords.get(0, 2)[0] * k, theta);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new Robot(hardwareMap, telemetry);
        cam = new Camera(hardwareMap, "webcam", 640, 480, tagpipeline);

        tagpipeline.begin();

        // if debug is 3, a tag was successfully detected, otherwise problems appeared
        while(opModeInInit()) {
            if(tagpipeline.debug == 3){
                int tagId = tagpipeline.getTagId();
                Pose2d deltaRcurrent = openCVMat2RRPose2d(tagpipeline.getXYZ(), tagpipeline.getTheta());
            }
        }

        waitForStart();

        bot.drive.trajectorySequenceBuilder(new Pose2d(0.1, 0, Math.toRadians(0)))
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
    }
}
