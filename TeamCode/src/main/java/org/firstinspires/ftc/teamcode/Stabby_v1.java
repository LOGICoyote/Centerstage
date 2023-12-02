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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.PIDRunner;

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
    private DcMotor Llifttest;

    private double slideoutconstant = 0.8;
    private PIDRunner rSlidePID;
    private PIDRunner lSlidePID;
    private double precisemovespeed=0.5;
    private double desSlideHeight = 0.0;

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
       // Llift = hardwareMap.get(DcMotor.class, "llift");
        Llifttest=hardwareMap.get(DcMotor.class, "xlift");
        Rlift = hardwareMap.get(DcMotor.class, "rlift");

        rSlidePID = new PIDRunner(0.03, 0, 0, getRuntime());
        lSlidePID = new PIDRunner(0.03, 0, 0, getRuntime());
        

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        Rlift.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void start() {
        runtime.reset();
        //Llift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Rlift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
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

        // lift controls
        if (gamepad1.a) {
            desSlideHeight = convertToCM(3.0);
        }
        if (gamepad1.b) {
            desSlideHeight = convertToCM(6.0);
        }
       // lift(desSlideHeight); // run this every cycle
        if (gamepad1.x){
            Llifttest.setPower(.5);
        }

    }

    @Override
    public void stop() {
    }

    private double convertToCM(double inches) {
        double cm = inches * 2.54;
        return cm;
    }

    private void lift(double liftpos) {

        double rPosition = Rlift.getCurrentPosition();
        double lPosition = Llift.getCurrentPosition();
        
        double rHeightCM = (3.18 * rPosition * Math.PI) / 145.1;
        double lHeightCM = (3.18 * lPosition * Math.PI) / 145.1;
        
        double rPower = rSlidePID.calculate(liftpos,  rHeightCM, getRuntime());
        double lPower = lSlidePID.calculate(liftpos, lHeightCM, getRuntime());

        double differentialHeight = Math.abs(rHeightCM - lHeightCM);
        double averageHeight = (rHeightCM + lHeightCM) / 2;
        if(differentialHeight > 0.3) {
            if(rHeightCM > lHeightCM) {
                rPower = rPower - 0.1;
            } else {
                lPower = lPower - 0.1;
            }
        }

        // double lift_power = (liftpos - Llift.getCurrentPosition()) * 0.002;
        //.002
        // Llift.setTargetPosition(liftpos);
        // if (Llift.getCurrentPosition() >= liftpos) {
        //     Llift.setPower(-1);
        //     Llift.setPower(-1);
        //     Rlift.setPower(-1);
        // }
        // Llift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // uncomment when ready to test
        Llift.setPower(-0.5);
        Rlift.setPower(-0.5);
        telemetry.addData("lift r pos", rHeightCM);
        telemetry.addData("lift l pos", lHeightCM);
        telemetry.addData("despos", liftpos);
        telemetry.addData("desheight1", desSlideHeight);
        telemetry.addData("lift L power", lPower);
        telemetry.addData("lift R power", rPower);
        telemetry.addData("encoderL", Llift.getCurrentPosition());
        telemetry.addData("encoderR", Rlift.getCurrentPosition());
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