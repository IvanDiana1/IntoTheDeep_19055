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
    public int MaxRange=1720;
    public int currentPos=0;
    private static double p=12 , i =0.4 , d =0.07;


    public static PIDCoefficients pidLift = new PIDCoefficients (p , i , d) ;
    //public static PIDFController pidCLift = new PIDFController();
    public static PIDFController pidCLift  =new PIDFController(pidLift);
    public static VoltageSensor voltage_sensor;
    public Lifteer (HardwareMap hardwareMap , Telemetry telemetry ){
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

        pidCLift = new PIDFController(pidLift);
    }

    public enum LIFTER_STATES{
       DOWN(0), UP(1720), MIDDLE(1500);
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

        currentPos = rlifter.getCurrentPosition();

        double power =pidCLift.update(((double)currentPos/MaxRange))*(14/ voltage_sensor.getVoltage());
        llifter.setPower(power);
        rlifter.setPower(power);

    }


    public void telemetryData(){
       telemetry.addData(" LIFTER Target: ",Target);
        telemetry.addData("LIFTER POS: ",rlifter.getCurrentPosition());
       // telemetry.addData("pow", power);
        //   telemetry.addData("PID: ", current_pid.);

    }
}
