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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp

public class CheetoV1 extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private RevColorSensorV3 dist;
    private double movespeed = 21;
    private double turnspeed = .75;
    private DcMotor turret = null;
    private Servo claw;
    private Servo holdL;
    private Servo holdR;
    private DcMotor lift = null;
    private Servo extend;
    private boolean Y_pressed = false;
    private int intakestop = 1;
    private boolean intakeon = false;
    private double precisemovespeed=0.5;
    private int liftpos =700;
    private boolean up_pressed = false;
    private boolean clawup = false;
    private int turretpos;
    private double turretpower;
    private boolean intakeopen = true;
    private RevBlinkinLedDriver leds;
    private double tensionfix = 1.1;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = hardwareMap.get(DcMotor.class, "motor00");
        rightFront = hardwareMap.get(DcMotor.class, "motor01");
        leftBack = hardwareMap.get(DcMotor.class, "motor02");
        rightBack = hardwareMap.get(DcMotor.class, "motor03");
        motor1 = hardwareMap.get(DcMotor.class, "motor 0");
        motor2 = hardwareMap.get(DcMotor.class, "motor 1");
        dist = hardwareMap.get(RevColorSensorV3.class,"dist");
        claw=hardwareMap.get(Servo.class,"claw");
        turret = hardwareMap.get(DcMotor.class, "turret");
        lift =  hardwareMap.get(DcMotor.class, "motor 3");
        holdL = hardwareMap.get(Servo.class,"holdL");
        holdR = hardwareMap.get(Servo.class,"holdR");
        extend = hardwareMap.get(Servo.class,"extendy");
        leds = hardwareMap.get(RevBlinkinLedDriver.class, "LEDS");




        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.REVERSE);
        turret.setDirection(DcMotor.Direction.REVERSE);
//        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void start() {
        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extend.setPosition(.35);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("dist", dist.getDistance(DistanceUnit.MM));
        telemetry.addData("This is your Robot", "I am staging a coup");
        telemetry.addData("lift", lift.getCurrentPosition());
        telemetry.addData("turretpos", turret.getCurrentPosition());
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

        if (left_trigger > 0) {
            turnleft(left_trigger);

        } else if (right_trigger > 0) {
            turnright(right_trigger);

        } else if (right_XAxis > 0.2) {
            straferight(right_XAxis * precisemovespeed);

        } else if (right_XAxis < -0.2) {
            strafeleft(right_XAxis * -precisemovespeed);

        } else if (right_YAxis > 0.2) {
            goforward(right_YAxis * precisemovespeed);

        } else if (right_YAxis < -0.2) {
            gobackward(right_YAxis * precisemovespeed);

        } else if (left_XAxis > 0.4) {
            straferight(left_XAxis);

        } else if (left_XAxis < -0.4) {
            strafeleft(-left_XAxis);

        } else if (left_YAxis > 0.2) {
            goforward(left_YAxis);

        } else if (left_YAxis < -0.2) {
            gobackward(left_YAxis);

        } else
            gostop(0);

        //intake controls
        if (gamepad1.right_bumper) {
            intakeforward();
        }
        if (gamepad1.b) {
            intakeoff();
        }
        if ((dist.getDistance(DistanceUnit.MM) < 42)) {
            holdL.setPosition(.65);
            holdR.setPosition(.35);
            //intakeoff();
            intakeopen = false;
        }
        if ((dist.getDistance(DistanceUnit.MM) > 42)) {
            holdL.setPosition(.5);
            holdR.setPosition(.5);
            intakeopen = true;
        }

        if (gamepad1.dpad_right) {
            extend.setPosition(.35);
        }
//        if(gamepad1.left_bumper){
//            lift.setPower(.3);
//        }
//        closed
//        if (gamepad1.dpad_right){
//            holdL.setPosition(.7);
//            holdR.setPosition(.3);
//        }
        //off/reverse toggle
        if (gamepad1.y) {
            if (!Y_pressed) {
                if (intakestop == 1) {
                    intakeoff();
                    intakestop = 0;
                } else {
                    intakereverse();
                    intakeon = false;
                    intakestop = 1;
                }
            }
            Y_pressed = true;
        } else
            Y_pressed = false;
//        if (gamepad1.dpad_up){
//            liftpos+=10;
//        }
//        if (gamepad2.dpad_up) {
//            if (!up_pressed){
//                liftpos+=10;
//            }
//            up_pressed = true;
//        } else
//            up_pressed = false;

        if (gamepad1.dpad_left) {
            extend.setPosition(.65);
        }
        if (gamepad1.a) {
            clawopen();
        }
        if (gamepad1.x) {
            clawclosed();
        }
        if (gamepad2.dpad_up) {
            clawclosed();
        }
        if (gamepad1.left_bumper) {claw_up();
        }
        if (gamepad1.dpad_up) {
            claw_up();
            //800
            lift(800);
            //clawup=true;
        }
        if (gamepad1.dpad_down) {
            lift(0);
            clawup = false;
        }
        if (intakeopen && (lift.getCurrentPosition() > 240)) {
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);

//        if (lift.getCurrentPosition()<240){
//            intakeforward();
//        }
        //2nd driver controls
        if (gamepad2.left_bumper) {
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turret.setPower(.3);
        }
        if (gamepad2.left_trigger > .2) {
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turret.setPower(.2);
            //turret.setPower(0);
        }
        if (gamepad2.right_bumper) {
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turret.setPower(-.3);
        }
        if (gamepad2.right_trigger > .2) {
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turret.setPower(-.2);
        }
        if (gamepad2.a) {
            turret.setPower(0);
        }
        if (gamepad2.b) {
            holdL.setPosition(.5);
            holdR.setPosition(.5);
            intakeoff();
            intakeopen = true;
        }
        if (gamepad2.dpad_left) {
            extend.setPosition(extend.getPosition() - 0.01);
        }
        if (gamepad2.dpad_right) {
            extend.setPosition(extend.getPosition() + 0.01);
        }
        if (gamepad2.x) {
            //low
            lift(320);
        }
        if (gamepad2.y) {

            lift(570);
        }
        if (gamepad2.dpad_down) {
            clawopen();
        }
    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    private void lift(int liftpos) {
        double lift_power = (liftpos - lift.getCurrentPosition()) * 0.002;
        //.002
        lift.setTargetPosition(liftpos);
        if (lift.getCurrentPosition() >= liftpos) {
            lift.setPower(-1);
        }
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(lift_power);
        telemetry.addData("lift power", lift_power);
        movespeed = 0.65;
        turnspeed = 0.5;
    }
    private void turretmov(int turretpos) {
        turret.setTargetPosition(turretpos);
        double turret_power=0;
        if (turret.getCurrentPosition()<=turretpos){
            turret_power = (turretpos - turret.getCurrentPosition()) * 0.002;
            //turretpower =.2;
        }
        else if (turret.getCurrentPosition()>=turretpos){
            //turretpower = -.2;
             turret_power = (turretpos - turret.getCurrentPosition()) * 0.002;
        }
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(turret_power);

    }
    private void claw_up(){
        //265 for comp
        lift(265);
        clawup = true;
    }

    private void clawopen(){
        //claw.setPosition(.4);
        claw.setPosition(.4);
    }
    private void clawclosed(){
        claw.setPosition(.55);
    }
    private void intakeforward() {
        motor1.setPower(-.7);
        motor2.setPower(.7);
    }
    private void intakeoff() {
        motor1.setPower(0);

        motor2.setPower(0);
    }
    private void intakereverse() {
        motor1.setPower(1);
        motor2.setPower(-1);
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