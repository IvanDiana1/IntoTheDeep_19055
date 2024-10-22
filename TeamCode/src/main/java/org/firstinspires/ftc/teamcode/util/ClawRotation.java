package org.firstinspires.ftc.teamcode.util;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class ClawRotation {
    private Servo clawRotation;

    public ClawRotation( HardwareMap hardwareMap){
        clawRotation = hardwareMap.get(Servo.class, "clawRotation");
        clawRotation.setDirection(Servo.Direction.REVERSE);
        clawRotation.setPosition(1);
    }
    public void rotate( double angle){
        clawRotation.setPosition(angle);
    }

}
