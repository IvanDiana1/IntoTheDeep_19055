package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Lifteer implements Updateable{
    public DcMotorEx llifter , rlifter;

    public Telemetry telemetry;
    public int Target;
    public static int MaxRange=4250;
    public int currentPos=0;
    private boolean motors_resetable;
    public static int autoPos = 0;
    private static boolean runPID = true;
    private static double p=30 , i =0.3 , d =1;


    public static PIDCoefficients pidLift = new PIDCoefficients (p , i , d) ;
    //public static PIDFController pidCLift = new PIDFController();
    public static PIDFController pidCLift  =new PIDFController(pidLift);
    public static VoltageSensor voltage_sensor;

    public Lifteer(HardwareMap hardwareMap, Telemetry telemetry) {
        rlifter = hardwareMap.get(DcMotorEx.class, HardwareConfig.LifterRight);
        llifter = hardwareMap.get(DcMotorEx.class, HardwareConfig.LifterLeft);
        voltage_sensor = hardwareMap.getAll(VoltageSensor.class).get(0);

        rlifter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        llifter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        rlifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        llifter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        rlifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        llifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rlifter.setDirection(DcMotorEx.Direction.FORWARD);
        llifter.setDirection(DcMotorEx.Direction.REVERSE);
        this.telemetry= telemetry;

        runPID = true;
        motors_resetable = true;

        pidCLift = new PIDFController(pidLift);
    }

    public enum LIFTER_STATES{
       DOWN(0), UP(4250), MIDDLE(2300),SPECIMEN(1625), AGATATED(1550), LOWMID(450);
       public final int val;
       LIFTER_STATES(int val){
           this.val = val;
       }
    }

    public void setTarget(int pos){

        Target = pos;
        pidCLift.setTargetPosition((double)Target/MaxRange);
        pidCLift.reset();

    }


    @Override
    public void update(){

        currentPos = rlifter.getCurrentPosition()+autoPos;

        if(!runPID)
            return;

        double power =pidCLift.update(((double)currentPos/MaxRange))*(12/ voltage_sensor.getVoltage());
        llifter.setPower(power);
        rlifter.setPower(power);
        telemetryData();

        if(Target == 0 && !isBusy() && Math.abs(currentPos)>5 && motors_resetable ){
            autoPos = 0;
            reset_motors();}

    }


    public void telemetryData(){
       telemetry.addData(" LIFTER Target: ",Target);
        telemetry.addData("LIFTER POS: ",currentPos);
       // telemetry.addData("pow", power);
        //   telemetry.addData("PID: ", current_pid.);

    }

    public void float_motors(){
        rlifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        llifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        runPID = false;
        rlifter.setPower(0);
        llifter.setPower(0);
    }

    public void reset_motors(){
        rlifter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        llifter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        rlifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        llifter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        rlifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        llifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rlifter.setDirection(DcMotorEx.Direction.FORWARD);
        llifter.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void set_motorsResetable(boolean val){
        motors_resetable = val;
    }

    public boolean isBusy(){
       return (Math.abs(currentPos - Target) > 45);
    }
    public void setAutoPos(int pos){
        autoPos = pos;
    }
}
