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
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class Arm implements Updateable{

    public DcMotorEx rightArm;
    public DcMotorEx leftArm;
    public Telemetry telemetry;

    public int Target=0;
    public int lastTarget = 0;
    public boolean runPid;
    public static final int MaxTarget = 2450;
    public static final double ticks_to_deg = 15.31;
    public static final double initialDeg = -60;

    public static PIDCoefficients pidf_upcoeff = new PIDCoefficients(2.7, 0.15,0.005);
   // public static PIDCoefficients pidf_downcoeff = new PIDCoefficients(2.7, 0,d);

    public static PIDCoefficients pidf_midcoeff = new PIDCoefficients(4.5, 0.2 , 0.2);
    public static double fUp=0.07, fDown = 0.16 ,kf= 0.01;
    public static PIDFController pidf_upcon  = new PIDFController(pidf_upcoeff);
    public static PIDFController pidf_midcon = new PIDFController(pidf_midcoeff);
    public static PIDFController pidf_con;
    public static VoltageSensor voltage_sensor;

    public  Arm(HardwareMap hardwareMap, Telemetry telemetry){
        rightArm = hardwareMap.get(DcMotorEx.class, HardwareConfig.ArmRight);
        leftArm = hardwareMap.get(DcMotorEx.class, HardwareConfig.ArmLeft);
        voltage_sensor = hardwareMap.getAll(VoltageSensor.class).get(0);

        rightArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftArm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightArm.setDirection(DcMotorEx.Direction.FORWARD);
        leftArm.setDirection(DcMotorEx.Direction.REVERSE);
        this.telemetry= telemetry;


        pidf_con = pidf_upcon;
        pidf_con.reset();
    }

    public void powerLimits(double power){
      if(lastTarget>Target )
       this.power = power * 0.4;
     // else this.power = power * 0.8;
    }


    public enum ARM_STATES{

        DOWN(1), UP(2450), MIDDLE(500);
        public  final int val;
        ARM_STATES(int val){
            this.val=val;
        }
    }


    public void setTarget ( int pos){
        lastTarget= Target;
        Target= pos;

        if(Target== ARM_STATES.MIDDLE.val){
            pidf_con = pidf_midcon;
        }
        else pidf_con = pidf_upcon;

        pidf_con.setTargetPosition((double)Target/MaxTarget);
        pidf_con.reset();


    }
    double power;

    @Override
    public void update(){

        int currentPos = rightArm.getCurrentPosition();

        power =pidf_con.update(((double)currentPos/MaxTarget));

        double cos = Math.cos(Math.toRadians(Target/ticks_to_deg-initialDeg)) * kf;
        power = (power + cos)*(14/ voltage_sensor.getVoltage());
//        powerLimits(power);
        leftArm.setPower(power);
        rightArm.setPower(power);

    }

    public void telemetryData(){
        telemetry.addData("Target: ",Target);
        telemetry.addData("Rotation: ",rightArm.getCurrentPosition());
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
