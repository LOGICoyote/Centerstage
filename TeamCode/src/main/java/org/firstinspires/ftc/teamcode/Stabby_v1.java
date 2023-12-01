/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp

public class Stabby_v1 extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private double movespeed = 21;
    private double turnspeed = .75;
    private DcMotor intake;
    private Servo Rslideshift;
    private Servo Lslideshift;
    private DcMotor Llift;
    private DcMotor Rlift;
    private double slideoutconstant = 0.8;

    private double precisemovespeed=0.5;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        rightFront= hardwareMap.get(DcMotor.class, "rf");
        intake  = hardwareMap.get(DcMotor.class, "intake");
        Lslideshift=hardwareMap.get(Servo.class, "Lshift");
        Rslideshift=hardwareMap.get(Servo.class, "Rshift");
        Llift = hardwareMap.get(DcMotor.class, "llift");
        Rlift = hardwareMap.get(DcMotor.class, "rlift");



        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("Name me", "or I will revolt");
        telemetry.update();
        //telemetry.addData("liftpos", liftpos);
        // telemetry.update();
        // Setup a variable for the joysticks and triggers
        double left_YAxis;
        double left_XAxis;
        double right_YAxis;
        double right_XAxis;
        double left_trigger;
        double right_trigger;

        left_YAxis = -gamepad1.left_stick_y;
        left_XAxis = gamepad1.left_stick_x;
        right_YAxis = -gamepad1.right_stick_y;
        right_XAxis = gamepad1.right_stick_x;
        left_trigger = gamepad1.left_trigger;
        right_trigger = gamepad1.right_trigger;

//        if (left_trigger > 0) {
//            turnleft(left_trigger);
//
//        } else if (right_trigger > 0) {
//            turnright(right_trigger);
//
//        } else if (right_XAxis > 0.2) {
//            straferight(right_XAxis * precisemovespeed);
//
//        } else if (right_XAxis < -0.2) {
//            strafeleft(right_XAxis * -precisemovespeed);
//
//        } else if (right_YAxis > 0.2) {
//            goforward(right_YAxis * precisemovespeed);
//
//        } else if (right_YAxis < -0.2) {
//            gobackward(right_YAxis * precisemovespeed);
//
//        } else if (left_XAxis > 0.4) {
//            straferight(left_XAxis);
//
//        } else if (left_XAxis < -0.4) {
//            strafeleft(-left_XAxis);
//
//        } else if (left_YAxis > 0.2) {
//            goforward(left_YAxis);
//
//        } else if (left_YAxis < -0.2) {
//            gobackward(left_YAxis);
//
//        } else
//            gostop(0);

        //intake controls
        if (gamepad1.right_bumper) {
            intakeforward();
        }
        if (gamepad1.left_bumper) {
            intakereverse();
        }
        if (gamepad1.dpad_up){
            intakeoff();
        }
        if (gamepad1.dpad_left){
            Lslideshift.setPosition(0);
            Rslideshift.setPosition(0);

        }
        if (gamepad1.dpad_right){
            Lslideshift.setPosition(slideoutconstant);
            Rslideshift.setPosition(slideoutconstant);

        }
    }

    @Override
    public void stop() {
    }
    private void lift(int liftpos) {
        double lift_power = (liftpos - Llift.getCurrentPosition()) * 0.002;
        //.002
        Llift.setTargetPosition(liftpos);
        if (Llift.getCurrentPosition() >= liftpos) {
            Llift.setPower(-1);
            Rlift.setPower(-1);
        }
        Llift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Llift.setPower(lift_power);
        Rlift.setPower(lift_power);
        telemetry.addData("lift L power", lift_power);
        movespeed = 0.65;
        turnspeed = 0.5;

    }
    private void intakeforward() {
        intake.setPower(.7);
    }
    private void intakeoff() {
        intake.setPower(0);

    }
    private void intakereverse() {
        intake.setPower(-.7);
    }
    private void gostop(double speed) {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
    private void goforward(double speed) {
        leftFront.setPower(speed*movespeed);
        leftBack.setPower(speed*movespeed);
        rightFront.setPower(speed*movespeed);
        rightBack.setPower(speed*movespeed);
    }
    private void gobackward(double speed) {
        leftFront.setPower(speed * movespeed);
        leftBack.setPower(speed * movespeed);
        rightFront.setPower(speed * movespeed);
        rightBack.setPower(speed * movespeed);
    }
    private void strafeleft(double speed) {
        leftFront.setPower(speed * -movespeed);
        leftBack.setPower(speed * movespeed);
        rightFront.setPower(speed * movespeed);
        rightBack.setPower(speed * -movespeed);
    }
    private void straferight(double speed) {
        leftFront.setPower(speed * movespeed);
        leftBack.setPower(speed * -movespeed);
        rightFront.setPower(speed * -movespeed);
        rightBack.setPower(speed * movespeed);
    }
    private void turnleft(double speed) {
        leftFront.setPower(speed * -turnspeed);
        leftBack.setPower(speed * -turnspeed);
        rightFront.setPower(speed * turnspeed);
        rightBack.setPower(speed * turnspeed);
    }
    private void turnright(double speed) {
        leftFront.setPower(speed * turnspeed);
        leftBack.setPower(speed * turnspeed);
        rightFront.setPower(speed * -turnspeed);
        rightBack.setPower(speed * -turnspeed);
    }

}