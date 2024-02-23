package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.PIDRunner;

@TeleOp

public class Stabby_v1 extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private PIDRunner rSlidePID;
    private PIDRunner lSlidePID;
    private PIDRunner climbPID;
    private Servo plane;


    private double slideoutconstant = 0.3;
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

    private Robot robot;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        robot = new Robot(this);
        robot.initialize();

        rSlidePID = new PIDRunner(0.01, 0, 0, getRuntime());
        lSlidePID = new PIDRunner(0.01, 0, 0, getRuntime());
        climbPID = new PIDRunner(0.005, 0.00, 0, getRuntime());

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        robot.start();
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
            robot.driveTrain.turnLeft(left_trigger);

        } else if (right_trigger > 0) {
            robot.driveTrain.turnRight(right_trigger);

        } else if (right_XAxis > 0.2) {
            robot.driveTrain.strafeRight(right_XAxis * precisemovespeed);

        } else if (right_XAxis < -0.2) {
            robot.driveTrain.strafeLeft(right_XAxis * -precisemovespeed);

        } else if (right_YAxis > 0.2) {
            robot.driveTrain.forward(right_YAxis * precisemovespeed);

        } else if (right_YAxis < -0.2) {
            robot.driveTrain.reverse(right_YAxis * precisemovespeed);

        } else if (left_XAxis > 0.4) {
            robot.driveTrain.strafeRight(left_XAxis);

        } else if (left_XAxis < -0.4) {
            robot.driveTrain.strafeLeft(-left_XAxis);

        } else if (left_YAxis > 0.2) {
            robot.driveTrain.forward(left_YAxis);

        } else if (left_YAxis < -0.2) {
            robot.driveTrain.reverse(left_YAxis);

        } else
            robot.driveTrain.stop();

        //intake controls
        if (gamepad1.right_bumper) {
            robot.intake.forward();
        }
        if (gamepad1.left_bumper) {
            robot.intake.reverse();
        }
        if (gamepad1.dpad_up){
            robot.intake.off();
        }

        if (gamepad1.dpad_down){
            desSlideHeight = convertToCM(0);
        }

        // lift controls
        if (gamepad1.a) {
            desSlideHeight = convertToCM(15.0);
        }

        if (gamepad1.y) {
            //closed
            robot.leftClaw.setPosition(1);
            robot.rightClaw.setPosition(.0);
        }
        if (gamepad1.x) {
            //open
            robot.leftClaw.setPosition(1-.4);
            robot.rightClaw.setPosition(.15);
        }

        //driver 2
        if (gamepad2.left_bumper) {
            if (!twodlpressed) {
                if (lclawclosed){
                    robot.leftClaw.setPosition(.6);
                    lclawclosed = false;
                }
                else {
                    robot.leftClaw.setPosition(1);
                    lclawclosed= true;
                }
            }
            twodlpressed=true;
        } else
            twodlpressed = false;

        if (gamepad2.right_bumper) {
            if (!twodrpressed) {
                if (rclawclosed){
                    robot.rightClaw.setPosition(.15);
                    rclawclosed = false;
                }
                else {
                    robot.rightClaw.setPosition(0.0);
                    rclawclosed= true;
                }
            }
            twodrpressed=true;
        } else
            twodrpressed = false;

        if (gamepad2.y){
            plane.setPosition(.5);
        }

        if (gamepad2.a){
            desWristAngle=0.56;
        }
        if (gamepad2.b){
            desWristAngle=0.3;
        }

        if (gamepad2.x){
            desWristAngle=0.43;
        }

        if (gamepad2.left_trigger>.02){
            robot.door.setPosition(0);
        }
        if (gamepad2.right_trigger>.02){
            robot.door.setPosition(1);
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
       double climbat = robot.climber.getCurrentPosition();
       double climbpower= climbPID.calculate(climbpos,climbat,getRuntime());
       robot.climber.setPower(climbpower);
        telemetry.addData   ("climb pos", climbpos);
        telemetry.addData("climb encoder", robot.climber.getCurrentPosition());
        telemetry.addData("des climb", desclimb);
    }

    private void wristPrePlanning(){
        double liftat = robot.leftLift.getCurrentPosition();
        double translation = desSlideHeight - liftat;
        if(translation > 10){
            if(liftat < 0.3){
                desWristAngle = .56;
            }
            if (liftat > 10){
                desWristAngle = 0.3;
            }
        } else if(translation < -10) {
            if ((liftat < 10)&&(liftat>3)){
                desWristAngle = 0.56;
            }
            if (liftat <= 3){
                desWristAngle = 0.43;
            }
        }

        if(liftat > 10) {
            setWristAngle(desWristAngle);
        }
        telemetry.addData("lwristset",desWristAngle);
        telemetry.update();
    }

    private void setWristAngle(double wristAngle){
        robot.leftClawTurn.setPosition(wristAngle);
        robot.rightClawTurn.setPosition(wristAngle);
    }

    private void lift(double liftpos) {

        double rPosition = -robot.rightLift.getCurrentPosition();
        double lPosition = robot.leftLift.getCurrentPosition();
        
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
        robot.leftLift.setPower(lPower);
        robot.rightLift.setPower(rPower);
        telemetry.addData("lift r pos", rHeightCM);
        telemetry.addData("lift l pos", lHeightCM);
        telemetry.addData("despos", liftpos);
        telemetry.addData("desheight1", desSlideHeight);
        telemetry.addData("lift L power", lPower);
        telemetry.addData("lift R power", rPower);
        telemetry.addData("encoderL", robot.leftLift.getCurrentPosition());
        telemetry.addData("encoderR", robot.rightLift.getCurrentPosition());
        robot.driveTrain.changeMoveSpeed(0.65);
        robot.driveTrain.changeTurnSpeed(0.5);
    }
}