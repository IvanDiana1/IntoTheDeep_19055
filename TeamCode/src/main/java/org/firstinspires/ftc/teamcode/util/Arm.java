package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.Controller;

import java.io.PipedInputStream;
import java.util.Timer;

@Config
public class Arm implements Updateable{

    public DcMotorEx rightArm;
    public TouchSensor touch;
    public DcMotorEx leftArm;
    public Telemetry telemetry;
    private static boolean resetTimer = false;
    private static boolean extend_once=  false;
    public ElapsedTime dttimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public int Target=0;
    public int lastTarget = 0;
    int currentPos = 0;
    public boolean runPid;
    public static final int MaxTarget = 2450;
    public static final double ticks_to_deg = 15.31;
    public static final double initialDeg = -60;

    public static PIDCoefficients pidf_upcoeff = new PIDCoefficients(3.6, 0.1,0.0045);
   // public static PIDCoefficients pidf_downcoeff = new PIDCoefficients(2.7, 0,d);

    public static PIDCoefficients pidf_midcoeff = new PIDCoefficients(20, 1 , 0.6);
    public static PIDCoefficients pidf_downmidcoeff = new PIDCoefficients(12,0.4,0.2);
    public static double fUp=0.07, fDown = 0.16 ,kf= 0.01;
    public static PIDFController pidf_upcon  = new PIDFController(pidf_upcoeff);
    public static PIDFController pidf_midcon = new PIDFController(pidf_midcoeff);
    public static PIDFController pidf_downmidcon = new PIDFController(pidf_downmidcoeff);
    public static PIDFController pidf_con;
    public static VoltageSensor voltage_sensor;
    private Linkage linkage;

    public  Arm(HardwareMap hardwareMap, Telemetry telemetry, Linkage linkage){
        rightArm = hardwareMap.get(DcMotorEx.class, HardwareConfig.ArmRight);
        leftArm = hardwareMap.get(DcMotorEx.class, HardwareConfig.ArmLeft);
        voltage_sensor = hardwareMap.getAll(VoltageSensor.class).get(0);

        rightArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftArm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightArm.setDirection(DcMotorEx.Direction.REVERSE);
        leftArm.setDirection(DcMotorEx.Direction.FORWARD);
         touch = hardwareMap.get(TouchSensor.class, "touchSensor");

        this.telemetry= telemetry;
        this.linkage = linkage;

        pidf_midcon.reset();
        pidf_upcon.reset();
        pidf_downmidcon.reset();
        pidf_con = pidf_upcon;
        pidf_con.setTargetPosition(Target);
        pidf_con.reset();
    }

    public void powerLimits(double power){
      if(lastTarget>Target )
       this.power = power * 0.4;
     // else this.power = power * 0.8;
    }


    public enum ARM_STATES{

        DOWN(1), UP(2450), MIDDLE(624);
        public  final int val;
        ARM_STATES(int val){
            this.val=val;
        }
    }


    public void setTarget ( int pos){
        lastTarget= Target;
        Target= pos;

        if(Target==ARM_STATES.MIDDLE.val){
            if (lastTarget<Target)
                pidf_con = pidf_midcon;
            else
                pidf_con = pidf_downmidcon;
        }
        else pidf_con = pidf_upcon;

        pidf_con.setTargetPosition((double)Target/MaxTarget);
        pidf_con.reset();

        resetTimer=false;
        extend_once = false;

    }
    double power;

    @Override
    public void update(){
        double dt = dttimer.seconds();
        currentPos = rightArm.getCurrentPosition();

        power =pidf_con.update(((double)currentPos/MaxTarget));

        double cos = Math.cos(Math.toRadians(Target/ticks_to_deg-initialDeg)) * kf;
        power = (power + cos)*(14/ voltage_sensor.getVoltage());
//        powerLimits(power);
        leftArm.setPower(power);
        rightArm.setPower(power);

        if (pidf_con==pidf_midcon||pidf_con==pidf_downmidcon){
            if ((currentPos/Target>0.2 && currentPos/Target<3) && !extend_once){
                extend_once = true;
                if (!linkage.isExtended)
                    linkage.linkageMove(Linkage.EXTEND_STATES.EXTEND);
            }

        }


        if(Target==1 && currentPos<0 && touch.getValue()==1){
            if (!resetTimer) {
                dttimer.reset();
                resetTimer = true;
                return;
            }
            if (dttimer.seconds()<0.27)
                return;
            rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            rightArm.setDirection(DcMotorEx.Direction.REVERSE);

            pidf_con=pidf_upcon;


            pidf_con.reset();

        }

    }
    public int getArmPos(){
        return currentPos;
    }

    public void telemetryData(){
        telemetry.addData("Target: ",Target);
        telemetry.addData("Rotation: ",currentPos);
        telemetry.addData("pow", power);
        //   telemetry.addData("PID: ", current_pid.);
        telemetry.addData("voltage ", voltage_sensor.getVoltage());
    }
    public void reset()
    {
        leftArm.setPower(0);
        rightArm.setPower(0);
        pidf_con.reset();

    }

}
