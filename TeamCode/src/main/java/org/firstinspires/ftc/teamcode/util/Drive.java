package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive {
    private DcMotorEx LF;
    private DcMotorEx LB;
    private DcMotorEx RF;
    private DcMotorEx RB;

    public Drive(HardwareMap hwmap) {
        LF = hwmap.get(DcMotorEx.class, HardwareConfig.LF);
        LB = hwmap.get(DcMotorEx.class, HardwareConfig.LB);
        RF = hwmap.get(DcMotorEx.class, HardwareConfig.RF);
        RB = hwmap.get(DcMotorEx.class, HardwareConfig.RB);

        LF.setDirection(DcMotorEx.Direction.REVERSE);
        LB.setDirection(DcMotorEx.Direction.REVERSE);
        RF.setDirection(DcMotorEx.Direction.FORWARD);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);

    }
    private void setPowers(double LFpower, double RFpower, double LBpower , double RBpower){
        double max = 1.0;
        max=Math.max(max,   Math.abs(LFpower));
        max=Math.max(max,   Math.abs(LBpower));
        max=Math.max(max,   Math.abs(RFpower));
        max=Math.max(max,   Math.abs(RBpower));
        LFpower /= max;
        LBpower /= max;
        RBpower /= max;
        RFpower /= max;

        LF.setPower(LFpower);
        LB.setPower(LBpower);
        RF.setPower(RFpower);
        RB.setPower(RBpower);
    }

    public void drive( double forward , double right, double rotate){
        double LFpower = forward + right+ rotate;
        double RFpower = forward - right - rotate;
        double LBpower = forward - right +rotate;
        double RBpower = forward + right - rotate;
        setPowers(LFpower, RFpower, LBpower, RBpower);
    }



}

