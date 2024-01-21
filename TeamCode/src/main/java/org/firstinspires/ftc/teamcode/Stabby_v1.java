

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
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

    private DcMotor intake;

    private DcMotor Llift;
    private DcMotor Rlift;
    private DcMotor climb;

    private Servo lclawturn;
    private Servo rclawturn;
    private Servo rclaw;
    private Servo lclaw;
    private Servo door;


    private PIDRunner rSlidePID;
    private PIDRunner lSlidePID;
    private PIDRunner climbPID;
    private Servo plane;


    private double slideoutconstant = 0.3;
    private double movespeed = 21;
    private double turnspeed = .75;
    private double precisemovespeed=0.5;
    private double desSlideHeight = 0.0;

    private double desWristAngle = 0.5;
    private double desclimb = 0.0;
    private boolean ypressed = false;
    private boolean clawclosed = true;
    private boolean dpaduppressed = false;
    private boolean dpaddownpressed = false;
    private boolean twodlpressed = false;
    private boolean twodrpressed = false;
    private boolean lclawclosed = true;
    private boolean rclawclosed = true;

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
        climb  = hardwareMap.get(DcMotor.class, "climb");
        Llift = hardwareMap.get(DcMotor.class, "llift");
        Rlift = hardwareMap.get(DcMotor.class, "rlift");
        lclawturn=hardwareMap.get(Servo.class, "lclawturn");
        rclawturn=hardwareMap.get(Servo.class, "rclawturn");
        lclaw=hardwareMap.get(Servo.class, "lclaw");
        rclaw=hardwareMap.get(Servo.class, "rclaw");
        door=hardwareMap.get(Servo.class, "door");
        plane = hardwareMap.get(Servo.class, "plane");

//.1 on both
        rSlidePID = new PIDRunner(0.01, 0, 0, getRuntime());
        lSlidePID = new PIDRunner(0.01, 0, 0, getRuntime());
        climbPID = new PIDRunner(0.005, 0.00, 0, getRuntime());


        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
       // intake.setDirection(DcMotor.Direction.REVERSE);
//        Rlift.setDirection(DcMotorSimple.Direction.REVERSE);
//        Llift.setDirection(DcMotorSimple.Direction.REVERSE);


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void start() {
        runtime.reset();
        Llift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Rlift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        climb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        climb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        climb.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Llift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Rlift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rclawturn.setDirection(Servo.Direction.REVERSE);
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
        if (gamepad1.left_bumper) {
            intakereverse();
        }
        if (gamepad1.dpad_up){
            intakeoff();
        }
//        if (gamepad1.dpad_left){
//            Lslideshift.setPosition(0);
//            Rslideshift.setPosition(1);
//        }
//        if (gamepad1.dpad_right){
//            Lslideshift.setPosition(.15);
//            Rslideshift.setPosition(.85);
//            Lslideshift.setPosition(slideoutconstant);
//            Rslideshift.setPosition(1-slideoutconstant);
//        }
        if (gamepad1.dpad_down){
            desSlideHeight = convertToCM(0);
        }
        // lift controls
        if (gamepad1.a) {
            desSlideHeight = convertToCM(15.0);
        }
//TODO decide do we want to keep?
//        if (gamepad2.dpad_right){
//            desWristAngle+=.01;
//        }
//        if (gamepad2.dpad_left){
//            desWristAngle-=.01;
//        }

        if (gamepad1.y) {
            //closed
            lclaw.setPosition(1);
           rclaw.setPosition(.0);
        }
        if (gamepad1.x) {
            //open
            lclaw.setPosition(1-.4);
            rclaw.setPosition(.15);
        }
//TODO determine if using
//        if (gamepad1.y) {
//            if (!ypressed) {
//                if (clawclosed){
//                    lclaw.setPosition(.4);
//                    rclaw.setPosition(.6);
//                    clawclosed = false;
//                    lclawclosed=false;
//                    rclawclosed= false;
//                }
//                else {
//                    lclaw.setPosition(0.25);
//                    rclaw.setPosition(.75);
//                    clawclosed= true;
//                    lclawclosed = true;
//                    rclawclosed = true;
//                }
//            }
//            ypressed=true;
//        } else
//            ypressed=false;

        //driver 2
        //todo fix positions
        if (gamepad2.left_bumper) {
            if (!twodlpressed) {
                if (lclawclosed){
                    lclaw.setPosition(.6);
                    lclawclosed = false;
                }
                else {
                    lclaw.setPosition(1);
                    lclawclosed= true;
                }
            }
            twodlpressed=true;
        } else
            twodlpressed = false;

        if (gamepad2.right_bumper) {
            if (!twodrpressed) {
                if (rclawclosed){
                    rclaw.setPosition(.15);
                    rclawclosed = false;
                }
                else {
                    rclaw.setPosition(0.0);
                    rclawclosed= true;
                }
            }
            twodrpressed=true;
        } else
            twodrpressed = false;

//        if (gamepad2.left_bumper) {
//            lclaw.setPosition(.4);
////        }
//        if (gamepad2.right_bumper) {
//            rclaw.setPosition(.6);
//        }
//TODO uncomment
//        if (gamepad2.a) {
//            lclawturn.getController().setServoPosition(3,.25);
//            rclawturn.getController().setServoPosition(1,.25);
////            rclawturn.setPosition(.75);
//        }
        if (gamepad2.y){
            plane.setPosition(.5);
        }

//                if (gamepad2.b) {
//            lclawturn.setPosition(0.5);
//            rclawturn.setPosition(0.5);
//        }
//        if (gamepad2.y) {
//            //up tilt
//            lclawturn.setPosition(.9);
//            rclawturn.setPosition(0.1);
//        }
        if (gamepad2.a){
            desWristAngle=0.56;
        }
        if (gamepad2.b){

desWristAngle=0.3;    }
        if (gamepad2.x){
            desWristAngle=0.43;
        }

        if (gamepad2.left_trigger>.02){
            door.setPosition(0);
        }
        if (gamepad2.right_trigger>.02){
            door.setPosition(1);
        }

        if (gamepad2.dpad_up) {
            if (!dpaduppressed) {
                desSlideHeight += convertToCM(1.0);
            }
            dpaduppressed=true;
        } else
            dpaduppressed=false;
        if (gamepad2.dpad_down) {
            if (!dpaddownpressed) {
                desSlideHeight -= convertToCM(1.0);
            }
            dpaddownpressed=true;
        } else
            dpaddownpressed=false;


        lift(desSlideHeight);
        // run this every cycle
        climb(desclimb);
        wristPrePlanning();

    }

    @Override
    public void stop() {
    }

    private double convertToCM(double inches) {
        double cm = inches * 2.54;
        return cm;
    }
    private void climb(double climbpos){ // NOOOOOOO ITS NOT THIS ONE
       double climbat = climb.getCurrentPosition();
       double climbpower= climbPID.calculate(climbpos,climbat,getRuntime());
       climb.setPower(climbpower);
        telemetry.addData   ("climb pos", climbpos);
        telemetry.addData("climb encoder", climb.getCurrentPosition());
        telemetry.addData("des climb", desclimb);
    }

    private void wristPrePlanning(){
        double liftat = Llift.getCurrentPosition();
        double translation = desSlideHeight - liftat;
        if(translation > 10){
            if(liftat < 0.3){
                desWristAngle = .56;
            }
            if (liftat > 10){
                desWristAngle = 0.3;
            }
        }else if(translation < -10) {
            if ((liftat < 10)&&(liftat>3)){
                desWristAngle = 0.56;
            }
            if (liftat <= 3){
                desWristAngle = 0.43;
            }
        }
//        lclawturn.setPosition(.5);
//        telemetry.addData("lwristPose",lclawturn.getController().getServoPosition(3));
//        telemetry.addData("rwristPose",rclawturn.getController().getServoPosition(1));
//        double height = climbat/100;
        //bottom .43
        //.3 .3 .56
        // 10 10 .3
        if(liftat > 10) {
            setWristAngle(desWristAngle);
        }
        telemetry.addData("lwristset",desWristAngle);
//        telemetry.addData("rwristset",rclawturn.getPosition());
        telemetry.update();
    }
    private void setWristAngle(double wristAngle){
        lclawturn.setPosition(wristAngle);
        rclawturn.setPosition(wristAngle);
    }
    private void lift(double liftpos) {

        double rPosition = -Rlift.getCurrentPosition();
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
        Llift.setPower(lPower);
        Rlift.setPower(rPower);
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
        intake.setPower(1);
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



