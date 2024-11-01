package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Lifter implements Updateable {
    private DcMotorEx left_rot;
    private DcMotorEx right_rot;
    private Telemetry telemetry;

    public int Target=0;
    public int lastTarget=0;
    private static final int MAX_RANGE = 220;


    public static double kF=0.2;
    public static final double tick_in_degrees = 0.35;
    public static double angle=0;
    public static final double initial_angle = -45;

    private VoltageSensor voltage_sensor;



    public static PIDCoefficients UP_PID = new PIDCoefficients(1.5,0.11,0.15);
    public static PIDCoefficients DOWN_PID = new PIDCoefficients(0.69,0.33,0.1);
    //public static PIDCoefficients START_PID = new PIDCoefficients(12.5,0.25,0.1);
       public static PIDCoefficients MIDDLE_DOWN_PID = new PIDCoefficients(0.45,0.3,0.02);
      public static PIDCoefficients MIDDLE_UP_PID = new PIDCoefficients(2,0.01,0.1);
    public  PIDFController current_pid = new PIDFController( DOWN_PID) ;
    public static PIDFController up_pid_controller = new PIDFController(UP_PI
    public static PIDFController down_pid_controller = new PIDFController(DOWN_PID);
      public static PIDFController middle_up_controller = new PIDFController(MIDDLE_UP_PID);
       public static PIDFController middle_down_controller = new PIDFController(MIDDLE_DOWN_PID);
    public static boolean runPid = true;
    public Lifter(HardwareMap hwmap, Telemetry telemetry){
        left_rot = hwmap.get(DcMotorEx.class, HardwareConfig.LEFTROTATION);
        right_rot = hwmap.get(DcMotorEx.class, HardwareConfig.RIGHTROTATION);

        left_rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_rot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_rot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left_rot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_rot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_rot.setDirection(DcMotorSimple.Direction.REVERSE);
        right_rot.setDirection(DcMotorSimple.Direction.FORWARD);

        this.telemetry = telemetry;
        up_pid_controller = new PIDFController(UP_PID);
        down_pid_controller = new PIDFController(DOWN_PID);

        voltage_sensor = hwmap.getAll(VoltageSensor.class).get(0);


    }

    public void setupLifter(){
        left_rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_rot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_rot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        current_pid.setTargetPosition((double)Target / MAX_RANGE);
    }

    private enum LIFTER_STATES{
        LOW(0),MID(400),HIGH(600);
        double val;
        LIFTER_STATES(double val) {
            this.val = val;
        }
    }

    public void setTarget(int position){
        Target = position;
        if (Target>right_rot.getCurrentPosition() && Target!=220)
        {  current_pid= middle_up_controller;

        }
        else
        if(Target>right_rot.getCurrentPosition())
        { current_pid = up_pid_controller;}
        else
            if(Target==0) {
                if (lastTarget == MAX_RANGE) {
                    current_pid = down_pid_controller;
                } else current_pid = middle_down_controller;
            }

        current_pid.setTargetPosition((double)Target / MAX_RANGE);
        current_pid.reset();
        lastTarget=Target;
    }

    public void update(){
        //current_pid.setTargetPosition((double)Target / MAX_RANGE);

        int currentPosition = right_rot.getCurrentPosition();
        angle = currentPosition * tick_in_degrees + initial_angle;

        if (runPid){
            double power = ((current_pid.update((double)currentPosition / MAX_RANGE) + Math.cos(Math.toRadians(angle))*kF)*(14/voltage_sensor.getVoltage()))*0.8;
            left_rot.setPower(power);
            right_rot.setPower(power);
        }
        else {
            left_rot.setPower(0);
            right_rot.setPower(0);
            current_pid.reset();
        }
    }

    public void telemetryData(){
        telemetry.addData("Target: ",Target);
        telemetry.addData("Rotation: ",right_rot.getCurrentPosition());
        telemetry.addData("Angle: ", angle);


    }

}
