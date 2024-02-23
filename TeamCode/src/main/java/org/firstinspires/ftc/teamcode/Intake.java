package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotor motor;
    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "intake");
    }

    public void forward() {
        motor.setPower(1);
    }
    public void off() {
        motor.setPower(0);
    }
    public void reverse() {
        motor.setPower(-.7);
    }
}