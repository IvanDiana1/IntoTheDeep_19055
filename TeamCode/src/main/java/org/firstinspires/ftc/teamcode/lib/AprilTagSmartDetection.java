package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.roadrunner.util.Angle;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.apriltag.*;
import org.opencv.calib3d.Calib3d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

/*
Camera Matrix (Intrinsic Parameters):
[[815.18353895   0.         336.13375632]
 [  0.         816.46593397 234.37512541]
 [  0.           0.           1.        ]]

Distortion Coefficients:
[[-2.01722275e-02  1.38887531e+00  3.30332373e-04 -5.11935765e-04
  -5.56527080e+00]]
 */


// is this actually even smart ???
public class AprilTagSmartDetection extends OpenCvPipeline {
    public final long detectorPtr;
    private static final int width_res = 640, height_res = 480;
    private boolean kill = false, start = false;
    public int debug = -1;

    public static final double TAGSIZE = 0.1016, FX = 822.317, FY = 822.317, CX = 319.495, CY = 242.502;

    public static final Mat cameraTranslation = new Mat(3, 1, CvType.CV_32F);
    static{
        cameraTranslation.put(0, 0, new double[]{0, 0, 0});
    }

    public static final Mat cameraRotation = new Mat(3, 1, CvType.CV_32F);
    static{
        cameraRotation.put(0, 0, new double[]{0, 0, 0});
    }

    public AprilTagSmartDetection(){
        this.detectorPtr = AprilTagDetectorJNI.createApriltagDetector("tag36h11", 1.5F, 2);

        coordsH.put(0, 0, new double[]{0, 0, 0, 1});

        if(this.detectorPtr == 0){
            this.kill = true;
        }
    }

    public void begin(){
        this.start = true;
    }

    public void kill(){
        this.kill = true;
        AprilTagDetectorJNI.releaseApriltagDetector(detectorPtr);
    }

    private Mat colorStream = new Mat();
    private long tagsPtrZarr;
    private MatOfPoint2f crazyMat = new MatOfPoint2f();
    private int tagId = -1;

    private int detectAllTags(Mat input){
        if(input.empty()){
            return 0;
        }

        input.copyTo(colorStream);
        //if(this.cropRect != null){
        //    Rect roi = new Rect(cropRect[0], cropRect[1], cropRect[2], cropRect[3]);
        //    input = input.submat(roi);
        //}
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2GRAY);

        // tagsPtr will be a zarray_t *
        tagsPtrZarr = AprilTagDetectorJNI.runApriltagDetector(detectorPtr, input.dataAddr(), width_res, height_res);

        if(tagsPtrZarr == 0){
            debug = 0;
            return 0;
        }

        // now we have a normal array of apriltag_detection_t *
        long[] tagsPtrsArray = ApriltagDetectionJNI.getDetectionPointers(tagsPtrZarr);

        if(tagsPtrsArray.length == 0){
            debug = 1;
            return 0;
        }

        if(tagsPtrsArray[0] == 0){
            debug = 2;
            return 0;
        }

        tagId = ApriltagDetectionJNI.getId(tagsPtrsArray[0]);
        double[][] c = ApriltagDetectionJNI.getCorners(tagsPtrsArray[0]);

        crazyMat.fromArray(
                new Point(c[0][0], c[0][1]),
                new Point(c[1][0], c[1][1]),
                new Point(c[2][0], c[2][1]),
                new Point(c[3][0], c[3][1])
                );

        ApriltagDetectionJNI.freeDetectionList(tagsPtrZarr);

        debug = 3;
        return 1;
    }

    // real frame of reference: O is at the center of the tag
    private static final MatOfPoint3f mat3dapriltag = new MatOfPoint3f(
            new Point3(-TAGSIZE/2, +TAGSIZE/2, 0d),
            new Point3(+TAGSIZE/2, +TAGSIZE/2, 0d),
            new Point3(+TAGSIZE/2, -TAGSIZE/2, 0d),
            new Point3(-TAGSIZE/2, -TAGSIZE/2, 0d)
    );
    private static final Mat intrinsicParams = new Mat(3, 3, CvType.CV_32F);
    static {
        intrinsicParams.put(0, 0, new double[] {FX, 0, CX, 0, FY, CY, 0, 0, 1});
    }
    private static final MatOfDouble distCoeffs = new MatOfDouble();
    static {
        distCoeffs.fromArray(new double[] {-0.0449369, 1.17277, 0, 0, -1.33244, 0, 0, 0});
    }
    public Mat rvec = new Mat(1, 3, CvType.CV_32F), tvec = new Mat(1, 3, CvType.CV_32F);
    public Mat rvec2 = new Mat(1, 3, CvType.CV_32F), tvec2 = new Mat(1, 3, CvType.CV_32F);
    private void computeTvecRvec(MatOfPoint2f mat2dcameraframe){
        Calib3d.solvePnP(
                mat3dapriltag,
                mat2dcameraframe,
                intrinsicParams,
                distCoeffs,
                rvec, tvec,
                false, Calib3d.SOLVEPNP_IPPE_SQUARE
        );

        Calib3d.solvePnPRefineLM(mat3dapriltag, mat2dcameraframe, intrinsicParams, distCoeffs, rvec, tvec);
    }

    public Mat buildTransformationMatrix(Mat _rvec, Mat _tvec){
        // rodrigues to get R = RxRyRz
        Mat rmat = new Mat(3, 3, CvType.CV_32F);
        Calib3d.Rodrigues(_rvec, rmat);

        // build transform matrix
            /*
            T = [ r11 r12 r13 x ]
                [ r21 r22 r23 y ]
                [ r31 r32 r33 z ]
                [ 0   0   0   1 ]
             */
        Mat transform = new Mat(4, 4, CvType.CV_32F);
        rmat.copyTo(transform.colRange(0, 3).rowRange(0, 3));
        _tvec.copyTo(transform.col(4).rowRange(0, 3));
        transform.put(4, 0, new double[]{0, 0, 0, 1});

        return transform;
    }

    public static final Mat originH = new Mat(4, 1, CvType.CV_32F);
    static {
        originH.put(0, 0, new double[]{0, 0, 0, 1});
    }

    public static final double YAW = 0, PITCH = 0, ROLL = 0;
    public static final Mat rvecCamera = new Mat(1, 3, CvType.CV_32F);
    static{
        rvecCamera.put(0, 0, new double[]{PITCH, YAW, ROLL});
    }
    public static final Mat rmatCamera = new Mat(3, 3, CvType.CV_32F);
    static {
        Calib3d.Rodrigues(rvecCamera, rmatCamera);
    }
    // transformation matrix from camera frame to robot frame
    public static final Mat transform2 = new Mat(4, 4, CvType.CV_32F);
    static {
        transform2.put(0, 0,
                new double[]{
                    rmatCamera.get(0, 0)[0], rmatCamera.get(0, 1)[0], rmatCamera.get(0, 2)[0], 1,
                    rmatCamera.get(1, 0)[0], rmatCamera.get(1, 1)[0], rmatCamera.get(1, 2)[0], 1,
                    rmatCamera.get(2, 0)[0], rmatCamera.get(2, 1)[0], rmatCamera.get(2, 2)[0], 1,
                    0                      , 0                      , 0                      , 1
                });
    }

    // THE OUTPUTS: ----
    public double theta;
    public Mat coordsH = new Mat(1, 4, CvType.CV_32F);
    // ----------------- + tagId which was defined earlier

    public static final Mat emptyMat = new Mat();
    @Override
    public Mat processFrame(Mat input) {
        if(input.empty()){
            return emptyMat;
        }

        if(start && ! kill){
            int ok = detectAllTags(input);

            if(ok == 0){
                return input;
            }

            computeTvecRvec(crazyMat);

            //Mat transform1 = buildTransformationMatrix(rvec, tvec);

            // get coordinates of april tag relative to the camera frame of reference
            //coordsH.put(0, 0, tvec.get(0, 0)[0]);
            //coordsH.put(1, 0, tvec.get(0, 1)[0]);
            //coordsH.put(2, 0, tvec.get(0, 2)[0]);
            //coordsH.matMul(transform2); // x y z of april tag with respect to robot

            //theta = Angle.norm(/*PITCH +*/ rvec.get(0, 2)[0]);

            //List<MatOfPoint> drawMat = Collections.singletonList(new MatOfPoint(crazyMat.toArray()));

            //Imgproc.polylines(input, drawMat, true, new Scalar(100, 100, 100));
        }

        return input;
    }

    public Mat getXYZ(){
        return coordsH;
    }

    public double getTheta(){
        return theta;
    }

    public int getTagId(){
        return tagId;
    }
}
