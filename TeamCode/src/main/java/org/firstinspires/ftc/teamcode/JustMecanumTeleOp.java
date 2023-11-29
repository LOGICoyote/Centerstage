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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/** `
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp

public class JustMecanumTeleOp extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;


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
        motor1 = hardwareMap.get(DcMotor.class, "motor 1");
        motor2 = hardwareMap.get(DcMotor.class, "motor 2");
        //motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        //imu = hardwareMap.get(Gyroscope.class, "imu");
        //color00 = hardwareMap.get(DistanceSensor.class, "color00");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        // Start the Sidewinder in an open position

        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
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

        if (left_trigger > 0) {
            turnleft(left_trigger);

        } else if (right_trigger > 0) {
            turnright(right_trigger);

        } else if (right_XAxis > 0.2) {
            straferight(right_XAxis * 0.5);

        } else if (right_XAxis < -0.2) {
            strafeleft(right_XAxis * -0.5);

        } else if (right_YAxis > 0.2) {
            goforward(right_YAxis * 0.5);

        } else if (right_YAxis < -0.2) {
            gobackward(right_YAxis * 0.5);

        } else if (left_XAxis > 0.2) {
            straferight(left_XAxis);

        } else if (left_XAxis < -0.2) {
            strafeleft(-left_XAxis);

        } else if (left_YAxis > 0.2) {
            goforward(left_YAxis);

        } else if (left_YAxis < -0.2) {
            gobackward(left_YAxis);

        } else
            gostop(0);

        if (gamepad1.a){
            motor1.setPower(-1);
            motor2.setPower(1);
        }
        if (gamepad1.y){
            motor1.setPower(1);
            motor2.setPower(-1);
        }
        if (gamepad1.b){
            motor1.setPower(0);
            motor2.setPower(0);
        }



    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    private void gostop(double speed) {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
    private void goforward(double speed) {
        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);
    }
    private void gobackward(double speed) {
        leftFront.setPower(speed * 0.75);
        leftBack.setPower(speed * 0.75);
        rightFront.setPower(speed * 0.75);
        rightBack.setPower(speed * 0.75);
    }
    private void strafeleft(double speed) {
        leftFront.setPower(speed * -0.75);
        leftBack.setPower(speed * 0.75);
        rightFront.setPower(speed * 0.75);
        rightBack.setPower(speed * -0.75);
    }
    private void straferight(double speed) {
        leftFront.setPower(speed * 0.75);
        leftBack.setPower(speed * -0.75);
        rightFront.setPower(speed * -0.75);
        rightBack.setPower(speed * 0.75);
    }
    private void turnleft(double speed) {
        leftFront.setPower(speed * -0.5);
        leftBack.setPower(speed * -0.5);
        rightFront.setPower(speed * 0.5);
        rightBack.setPower(speed * 0.5);
    }
    private void turnright(double speed) {
        leftFront.setPower(speed * 0.5);
        leftBack.setPower(speed * 0.5);
        rightFront.setPower(speed * -0.5);
        rightBack.setPower(speed * -0.5);
    }

}