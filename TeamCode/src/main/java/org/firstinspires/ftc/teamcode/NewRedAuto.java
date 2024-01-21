package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.PIDRunner;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous

public class NewRedAuto extends LinearOpMode {

    OpenCvCamera WebCam;
    //Thing myThing;
    static int pos = 0;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    private Servo claw;
    private Servo extend;
    private double movespeed = 21;
    private double turnspeed = .75;
    private PIDRunner rSlidePID;
    private PIDRunner lSlidePID;
    private DcMotor intake;

    private DcMotor Llift;
    private DcMotor Rlift;
    private Servo lclawturn;
    private Servo rclawturn;
    private Servo rclaw;
    private Servo lclaw;
//    private Servo door;
    private double slideoutconstant = 0.3;
    Thing myThing;

    SharedHeightPos heightOb;

    LiftThread liftThread;


    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        lclaw=hardwareMap.get(Servo.class, "lclaw");
        rclaw=hardwareMap.get(Servo.class, "rclaw");
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        rightFront= hardwareMap.get(DcMotor.class, "rf");
        intake  = hardwareMap.get(DcMotor.class, "intake");

        Llift = hardwareMap.get(DcMotor.class, "llift");
        Rlift = hardwareMap.get(DcMotor.class, "rlift");
        lclawturn=hardwareMap.get(Servo.class, "lclawturn");
        rclawturn=hardwareMap.get(Servo.class, "rclawturn");
        lclaw=hardwareMap.get(Servo.class, "lclaw");
        rclaw=hardwareMap.get(Servo.class, "rclaw");
//        door=hardwareMap.get(Servo.class, "door");

        rSlidePID = new PIDRunner(0.1, 0, 0, getRuntime());
        lSlidePID = new PIDRunner(0.1, 0, 0, getRuntime());

        heightOb = new SharedHeightPos();
        liftThread = new LiftThread();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        myThing = new Thing();
        WebCam.setPipeline(myThing);
        WebCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                WebCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35, -60, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);
        TrajectorySequence traja1= drive.trajectorySequenceBuilder(startPose)
                .forward(25)
                .strafeLeft(3)
                .build();
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
              .forward(36)
                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .forward(10)
                .build();
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .forward(5)
                .build();
        TrajectorySequence trajc1= drive.trajectorySequenceBuilder(startPose)
                .forward(25)
                .turn(Math.toRadians(-90))
                .forward(10)
                .build();
        waitForStart();
        liftThread.start();

        sleep(3000);
        telemetry.addData("Analysis", myThing.getAnalysis());
        telemetry.addData("position", myThing.position);
        telemetry.update();
           if (myThing.position == Thing.CapPosition.C){
               //right
               drive.followTrajectorySequence(trajc1);
//               door.setPosition(1);
               drive.followTrajectorySequence(traj2);
                sleep(999999);            }
             else if (myThing.position == Thing.CapPosition.B){
                 //center
                drive.followTrajectorySequence(traj1);
//               door.setPosition(1);
               sleep(500);
               drive.followTrajectorySequence(traj2);
               sleep(1000);
               drive.followTrajectorySequence(traj3);
               sleep(999999);
             }
            else if (myThing.position == Thing.CapPosition.A){
                //left
               drive.followTrajectorySequence(traja1);
//               door.setPosition(1);
               sleep(500);
               drive.followTrajectorySequence(traj2);

               sleep(999999);
           }
    }

    private void lift(double liftpos) {
        heightOb.setHeight(liftpos);

    }

    private double convertToCM(double inches) {
        double cm = inches * 2.54;
        return cm;
    }

    private void clawdump() {
        lclawturn.setPosition(0.3);
        rclawturn.setPosition(0.3);
    }
    private void clawopen(){
        lclaw.setPosition(1-.4);
        rclaw.setPosition(.15);
    }
    private void intakeonslow() {
        intake.setPower(0.2);
    }
    private void intakeoff() {
        intake.setPower(0);
    }
    private void clawready() {
        lclawturn.setPosition(.5);
        rclawturn.setPosition(.5);
    }
    
    public static class Thing extends OpenCvPipeline {
        public enum CapPosition {
            A,
            B,
            C
        }

        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Point TopLeftPoint = new Point(20, 75);
        static final Point BTopLeftPoint = new Point(170, 75);
        static final Point CTopLeftPoint = new Point(250, 75);
        static final int Region_width = 50;
        static final int Region_height = 70;
        Point region1_pointA = new Point(TopLeftPoint.x, TopLeftPoint.y);
        Point region1_pointB = new Point(TopLeftPoint.x + Region_width, TopLeftPoint.y + Region_height);
        Point region2_pointA = new Point(BTopLeftPoint.x, BTopLeftPoint.y);
        Point region2_pointB = new Point(BTopLeftPoint.x + Region_width, BTopLeftPoint.y + Region_height);
        Point region3_pointA = new Point(CTopLeftPoint.x, CTopLeftPoint.y);
        Point region3_pointB = new Point(CTopLeftPoint.x + Region_width, CTopLeftPoint.y + Region_height);
        Mat region1_Cr;
        Mat region2_Cr;
        Mat region3_Cr;

        Mat YCrCb = new Mat();
        Mat Cr = new Mat();
        Mat Cb = new Mat();
        int avg1;
        int avg2;
        int avg3;
        private volatile CapPosition position = CapPosition.A;
        private volatile CapPosition positionb = CapPosition.B;
        private volatile CapPosition positionc = CapPosition.C;

        void inputToCb(Mat input){
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb,Cb,1);
        }
        @Override
        public void init (Mat firstFrame){
           inputToCb(firstFrame);
            region1_Cr= Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cr= Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cr= Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);
            avg1 = (int)Core.mean(region1_Cr).val[0];
            avg2 = (int)Core.mean(region2_Cr).val[0];
            avg3 = (int)Core.mean(region3_Cr).val[0];

//            Imgproc.rectangle(input, region1_pointA, region1_pointB, BLUE, 2);
            Imgproc.rectangle(input, region2_pointA, region2_pointB, BLUE, 2);
            Imgproc.rectangle(input, region3_pointA, region3_pointB, BLUE, 2);

            position = CapPosition.A;
            if ((avg1>=avg2)&&(avg1>=140)){
                position = CapPosition.A;
            }
            else if ((avg2>avg1)&&(avg2>130)){
                position = CapPosition.B;
            }
            else {
            position = CapPosition.C;
            }
            Imgproc.rectangle(input,region1_pointA,region1_pointB, GREEN, 1);
            return input;
        }
        public int getAnalysis(){
            return avg1;
        }
    }

    public class SharedHeightPos {
        public volatile double height = 0;

        public double getHeight() {
            return height;
        }

        public void setHeight(double height) {
            this.height = height;
        }
    }

    public class LiftThread extends Thread
    {
        DcMotor Llift;
        DcMotor Rlift;

        SharedHeightPos heightOb;

        public LiftThread()
        {
            this.setName("DriveThread");
            Llift = hardwareMap.get(DcMotor.class, "llift");
            Rlift = hardwareMap.get(DcMotor.class, "rlift");

            heightOb = new SharedHeightPos();
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {

            try
            {
                while (!isInterrupted())
                {
                    // we record the Y values in the main class to make showing them in telemetry
                    // easier.

                    double rPosition = -Rlift.getCurrentPosition();
                    double lPosition = Llift.getCurrentPosition();

                    double rHeightCM = (3.18 * rPosition * Math.PI) / 145.1;
                    double lHeightCM = (3.18 * lPosition * Math.PI) / 145.1;
                    double liftpos = heightOb.getHeight();
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
//        telemetry.addData("desheight1", );
                    telemetry.addData("lift L power", lPower);
                    telemetry.addData("lift R power", rPower);
                    telemetry.addData("encoderL", Llift.getCurrentPosition());
                    telemetry.addData("encoderR", Rlift.getCurrentPosition());
                    movespeed = 0.65;
                    turnspeed = 0.5;
                    idle();
                }
            }
            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
//            catch (InterruptedException e) {System.out.println("%s interrupted");}
            // an error occurred in the run loop.
            catch (Exception e) {System.out.println(e);}


        }
    }


}