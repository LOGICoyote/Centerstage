package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {
    private double moveSpeed = 0.65;
    private double turnSpeed = 0.5;
    private final DcMotor leftFront;
    private final DcMotor rightFront;
    private final DcMotor leftBack;
    private final DcMotor rightBack;

    public DriveTrain(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void changeMoveSpeed(double speed) {
        moveSpeed = speed;
    }

    public void changeTurnSpeed(double speed) {
        turnSpeed = speed;
    }

    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
    public void forward(double speed) {
        leftFront.setPower(speed * moveSpeed);
        leftBack.setPower(speed * moveSpeed);
        rightFront.setPower(speed * moveSpeed);
        rightBack.setPower(speed * moveSpeed);
    }
    public void reverse(double speed) {
        leftFront.setPower(speed * moveSpeed);
        leftBack.setPower(speed * moveSpeed);
        rightFront.setPower(speed * moveSpeed);
        rightBack.setPower(speed * moveSpeed);
    }
    public void strafeLeft(double speed) {
        leftFront.setPower(speed * -moveSpeed);
        leftBack.setPower(speed * moveSpeed);
        rightFront.setPower(speed * moveSpeed);
        rightBack.setPower(speed * -moveSpeed);
    }
    public void strafeRight(double speed) {
        leftFront.setPower(speed * moveSpeed);
        leftBack.setPower(speed * -moveSpeed);
        rightFront.setPower(speed * -moveSpeed);
        rightBack.setPower(speed * moveSpeed);
    }
    public void turnLeft(double speed) {
        leftFront.setPower(speed * -turnSpeed);
        leftBack.setPower(speed * -turnSpeed);
        rightFront.setPower(speed * turnSpeed);
        rightBack.setPower(speed * turnSpeed);
    }
    public void turnRight(double speed) {
        leftFront.setPower(speed * turnSpeed);
        leftBack.setPower(speed * turnSpeed);
        rightFront.setPower(speed * -turnSpeed);
        rightBack.setPower(speed * -turnSpeed);
    }
}