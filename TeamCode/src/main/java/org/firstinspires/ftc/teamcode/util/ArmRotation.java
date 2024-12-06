package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ArmRotation implements Updateable{
    private DcMotorEx leftRot;
    private DcMotorEx rightRot;
    public static int Target;
    private static final int MAX_TARGET = 400;
    private static PIDCoefficients pid_coefficients;
    private static PIDFController rot_pid;
    public ArmRotation(HardwareMap hwmap, Telemetry telemetry){
//        leftRot = hwmap.get(DcMotorEx.class, HardwareConfig.ArmRotationLeft);
//        rightRot = hwmap.get(DcMotorEx.class, HardwareConfig.ArmRotationRight);

        leftRot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftRot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRot.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRot.setDirection(DcMotorSimple.Direction.REVERSE);


        pid_coefficients = new PIDCoefficients(3,0.2,0.1);
        rot_pid = new PIDFController(pid_coefficients);
    }
    public void setTarget(int target){
        Target = target;
        rot_pid.reset();
        rot_pid.setTargetPosition((double)Target/MAX_TARGET);

    }
    @Override
    public void update(){
        int current_position = leftRot.getCurrentPosition();
        double pow = rot_pid.update((double)current_position/MAX_TARGET);
        leftRot.setPower(pow);
        rightRot.setPower(pow);
    }
}
