/* Copyright Lydia & M1
Take it, I don't care
Programming is just copying
 */


package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp
public class CheddarV10 extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    //Hardware
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotorEx arm = null;
    private DcMotor intake;
    private Servo mailbox;
    //bort is name of tape measure cap mechanism
//    private Servo bort_x;
//    private Servo bort_y;
   // private CRServo bort_extend;
    private Servo OL;
    private Servo OR;
    private Servo OS;
    //private Rev2mDistanceSensor dist;
    private RevColorSensorV3 dist;

    //Variables
    private boolean cargointook = false;
    private boolean holding = false;
    private int DpadPos = 0;
    private double movespeed = 1;
    private double turnspeed = 0.75;
    private  int Rightpressed = 0;
    private int intakestop = 1;
    private int a = 0;
    private double holdtime = 0.0;
    private double spitdelay = 0.1;
    private int num = 0;
    private boolean responded = false;
    private boolean intakeon = false;
    private int armpos = 435;
    private boolean endgamemode = false;
//	private double bort_precisionY = 0.025;
//    private double bort_precisionX = 0.025;
    private boolean automationdisabled = false;
    private boolean R_pressed = false;
    private boolean dl = false;
    private boolean dr= false;
    private boolean du= false;
    private boolean dd= false;
    private double duckspeed = 0.35;
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        //Drive Motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftRear");
        rightBack = hardwareMap.get(DcMotor.class, "rightRear");
        arm = hardwareMap.get(DcMotorEx.class, "arm");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDCoefficients(12,0.1,0.75));

      //Odometry
        OR = hardwareMap.get(Servo.class, "OR");
        OL = hardwareMap.get(Servo.class, "OL");
        OS = hardwareMap.get(Servo.class, "OS");
        OL.setPosition(1);
        OR.setPosition(0);
        OS.setPosition(0);

        //implements
        mailbox = hardwareMap.get(Servo.class, "mailbox");
        intake = hardwareMap.get(DcMotor.class, "intake");
        dist = hardwareMap.get(RevColorSensorV3.class, "dist");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //bort
//        bort_x = hardwareMap.get(Servo.class, "bx");
//        bort_y = hardwareMap.get(Servo.class, "by");
//        bort_extend = hardwareMap.get(CRServo.class, "be");

        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void start() {
        runtime.reset();
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        accepting();
//        bort_x.setPosition(1);
//        bort_y.setPosition(0.65);
    }

    @Override
    public void loop() {
        telemetry.addData("Mailbox",mailbox.getPosition());
        telemetry.addData("dist",dist.getDistance(DistanceUnit.MM));
      /*  telemetry.addData("light", dist.getLightDetected());
        telemetry.addData("alpha", dist.alpha());
        telemetry.addData("blue", dist.blue());
        telemetry.addData("argb", dist.argb());*/
        telemetry.addData("engame", endgamemode);
        telemetry.addData("This is your Robot", "I am staging a coup");
        telemetry.addData("arm pid things", arm.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        double left_YAxis = -gamepad1.left_stick_y;;
        double left_XAxis = gamepad1.left_stick_x;
        double right_YAxis = -gamepad1.right_stick_y;
        double right_XAxis = gamepad1.right_stick_x;
        double left_trigger = gamepad1.left_trigger;
        double right_trigger = gamepad1.right_trigger;



        //Mecanum Drive
        if (left_trigger > 0) {
            turnleft(left_trigger);

        } else if (right_trigger > 0) {
            turnright(right_trigger);

        }
        else if ((right_XAxis > 0.35 ) && (!endgamemode)) {
            straferight(right_XAxis * 0.5);

        } else if ((right_XAxis < -0.35)&& (!endgamemode)) {
            strafeleft(right_XAxis * -0.5);

        } else if ((right_YAxis > 0.2)&& (!endgamemode)) {
            goforward(right_YAxis * 0.5);

        } else if ((right_YAxis < -0.2)&& (!endgamemode)) {
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

        //Automation
       if ((dist.getDistance(DistanceUnit.MM) < 30) && !cargointook && !responded && intakeon){
            cargointook = true;
           // holding();
            intake.setPower(-0.6);
            holdtime = getRuntime();
            responded = true;
        }
       if ((dist.getDistance(DistanceUnit.MM)<30) && (mailbox.getPosition()>0.075)){
           gamepad1.rumble(0,0.5,500);
       }
        if (cargointook && (getRuntime() >= holdtime + spitdelay)){
            holding();
            cargointook = false;
            holding = false;
            responded = false;
        }
        if ( (dist.getDistance(DistanceUnit.MM) > 80)&& (intakeon)){
            //intake.setPower(0);
            cargointook = false;
            holding = false;
            responded = false;
        }
       if (cargointook && (getRuntime() >= holdtime + spitdelay)){
            //intake.setPower(0);
            cargointook = false;
            holding = false;
            responded = false;
        }

        //if (dist.getDistance(DistanceUnit.MM))
        // intake on
        if((gamepad1.x)&&(!endgamemode)) {
            intake();
            //intake.setPower(1);
        }
        // intake reverse toggle
        if ((gamepad1.b)&&(!endgamemode)) {
            if (Rightpressed == 0) {
                if (intakestop == 1){
                    intake.setPower(-0.6);
                    intakestop = 0;
                }
                else {
                    intake.setPower(0);
                    intakeon= false;
                    intakestop =1;
                }
            }
            Rightpressed = 1;
        } else
            Rightpressed = 0;

        //arm up
        if (gamepad1.right_bumper){
           armup();
       }
       //arm down with mailbox to accepting
        if (gamepad1.left_bumper){
          armdown();
        }
        //mailbox
        //accepting
        if ((gamepad1.dpad_up)&&(!endgamemode)){
        accepting();
        }
        //dump
        if ((gamepad1.dpad_down)&&(!endgamemode)){
        dump();
        //accepting();
        }
        if ((gamepad1.dpad_left)&&(!endgamemode)){
            holding();
        }
        //bort code
        //TODO: Tune bort limits
//        if ((gamepad1.dpad_left)&&(endgamemode)&& (bort_x.getPosition()>= 0.6)) {
//            if (!dl) {
//                bort_x.setPosition(bort_x.getPosition()-bort_precisionX);
//            }
//            dl = true;
//        } else
//            dl = false;
//
//        if ((gamepad1.dpad_right)&&(endgamemode)&& (bort_x.getPosition()<=0.95)) {
//            if (!dr) {
//                bort_x.setPosition(bort_x.getPosition()+bort_precisionX);
//            }
//            dr = true;
//        } else
//            dr = false;
//        if ((gamepad1.dpad_up)&&(endgamemode)&&(bort_y.getPosition()>=0.3)) {
//            if (!du) {
//                bort_y.setPosition(bort_y.getPosition()-bort_precisionY);
//            }
//            du = true;
//        } else
//            du = false;
//        if ((gamepad1.dpad_down)&&(endgamemode)&&(bort_y.getPosition()<=0.6)) {
//            if (!dd) {
//                bort_y.setPosition(bort_y.getPosition()+bort_precisionY);
//            }
//            dd = true;
//        } else
//            dd = false;
//        if ((gamepad1.y)&&(endgamemode)) {
//			bort_precisionY = 0.05;
//			bort_precisionX = 0.05;
//		}
//        if ((gamepad1.x)&&(endgamemode)) {
//			bort_precisionY = 0.025;
//			bort_precisionX = 0.025;
//		}
//        if ((gamepad1.a)&&(endgamemode)) {
//			bort_precisionY = 0.01;
//			bort_precisionX = 0.01;
//		}
//        if ((gamepad1.dpad_left)&&(endgamemode)/*&& (bort_x.getPosition()<.9)&& (bort_x.getPosition()>.1)*/){
//            bort_x.setPosition(bort_x.getPosition()-0.05);
//        }
//        if ((gamepad1.dpad_right)&&(endgamemode)/*&& (bort_x.getPosition()<.9)&& (bort_x.getPosition()>.1)*/){
//            bort_x.setPosition(bort_x.getPosition()+0.05);
//        }
//        if ((gamepad1.dpad_up)&&(endgamemode)/*&& (bort_y.getPosition()<.9)&& (bort_y.getPosition()>.1)*/){
//            bort_y.setPosition(bort_y.getPosition()+0.05);
//        }
//        if ((gamepad1.dpad_down)&&(endgamemode)/*&& (bort_y.getPosition()<.9)&& (bort_y.getPosition()>.1)*/){
//            bort_y.setPosition(bort_y.getPosition()-0.05);
//        }
//        telemetry.addData("bort_x", bort_x.getPosition());
//        telemetry.addData("bort_y", bort_y.getPosition());
        //telemetry.update();

//        if ((right_YAxis >= 0.15)&&(endgamemode)){
//            bort_extend.setPower(-right_YAxis);
//        }
//        else if ((right_YAxis <= -0.15)&&(endgamemode)) {
//            bort_extend.setPower(-right_YAxis);
//
//        }
//        else {
//            bort_extend.setPower(0);
//        }

            //driver 2 controls

        //`arm` up
        if (gamepad2.right_bumper) {
           armup();
        }
        //arm down with mailbox move
        if (gamepad2.left_bumper){
          armdown();
        }

        //mailbox
        if ((gamepad2.y)&&(!endgamemode)){
            dump();
            //accepting();
        }
        if ((gamepad2.y)&&(endgamemode)){
            duckspeed = 0.2;
        }
        if (gamepad2.a){
            accepting();
        }
        if (gamepad2.dpad_up){
            armpos = 435;
        }
        if(gamepad2.dpad_left){
            armpos = 550;
        }
        if(gamepad2.dpad_down){
            armpos = 650;
        }
        /*if((gamepad2.dpad_right)&& (!endgamemode)){
            endgamemode = true;
        }*/
        if (gamepad2.dpad_right) {
            if (!R_pressed){
                if (!endgamemode){
                    endgamemode = true;
//                    bort_x.setPosition(0.6);
//                    bort_y.setPosition(0.4);
                }
                else {
                    endgamemode = false;
                }
            }
            R_pressed = true;
        } else
            R_pressed = false;

        /*if((gamepad2.dpad_right)&& (endgamemode)){
            endgamemode = false;
        }*/
       //TODO:Test intake - Duck speed
        //blue duck spin
        if (gamepad2.x){
            intake.setPower(duckspeed);
        }
      //  else intake.setPower(0);
        //red duck spin
        if (gamepad2.b){
            intake.setPower(-duckspeed);
        }
        if (gamepad2.right_trigger>0.2){
            intake.setPower(0);
            intakeon = false;
        }
        if (gamepad2.left_trigger>0.2){
            intake.setPower(-0.6);
            intakeon = false;
        }
        if (gamepad2.right_stick_button){

        }

    }


    @Override
    public void stop() {
    }
    private void armdown(){
        accepting();
        arm.setTargetPosition(0);
        if (arm.getCurrentPosition() <= 0) {
            arm.setPower(0);
        }
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.4);
        reset();
        movespeed = 1;
        turnspeed = 0.75;
    }
    private void armup() {
        double arm_power = (armpos - arm.getCurrentPosition()) * 0.002;
        arm.setTargetPosition(armpos);
        if (arm.getCurrentPosition() >= armpos) {
            arm.setPower(-1);
        }
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(arm_power);
        telemetry.addData("armpower", arm_power);
        movespeed = 0.65;
        turnspeed = 0.5;
        holding();
    }
    private void holding(){
        //0.7?
        mailbox.setPosition(0.1);
        //holding = true;
        num = 1;
    }
    private void intake(){
        accepting();
        intake.setPower(1);
        intakeon = true;
    }
    private void dump(){
        mailbox.setPosition(0.4);
        //num = 0;
    }
    private void accepting(){
        mailbox.setPosition(0);
        //num = 0;
    }
    private void reset(){
        //mailbox.setPosition(1);
        num = 0;
    }
    private void gostop(double speed) {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
    private void goforward(double speed) {
        leftFront.setPower(speed * movespeed);
        leftBack.setPower(speed * movespeed);
        rightFront.setPower(speed * movespeed);
        rightBack.setPower(speed * movespeed);
    }
    private void gobackward(double speed) {
        leftFront.setPower(speed * movespeed);
        leftBack.setPower(speed * movespeed);
        rightFront.setPower(speed * movespeed);
        rightBack.setPower(speed * movespeed);
    }
    private void strafeleft(double speed) {
        leftFront.setPower(speed * -1 * movespeed);
        leftBack.setPower(speed * movespeed);
        rightFront.setPower(speed * movespeed);
        rightBack.setPower(speed * -1 * movespeed);
    }
    private void straferight(double speed) {
        leftFront.setPower(speed * movespeed);
        leftBack.setPower(speed * -1 * movespeed);
        rightFront.setPower(speed * -1 * movespeed);
        rightBack.setPower(speed * movespeed);
    }
    private void turnleft(double speed) {
        leftFront.setPower(speed *  -1 * turnspeed);
        leftBack.setPower(speed * -1 * turnspeed);
        rightFront.setPower(speed * turnspeed);
        rightBack.setPower(speed * turnspeed);
    }
    private void turnright(double speed) {
        leftFront.setPower(speed * turnspeed);
        leftBack.setPower(speed * turnspeed);
        rightFront.setPower(speed *  -1 * turnspeed);
        rightBack.setPower(speed *  -1 * turnspeed);
    }

}