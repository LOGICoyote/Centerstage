package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    public final DcMotor leftLift;
    public final DcMotor rightLift;
    public final DcMotor climber;
    public final Servo leftClawTurn;
    public final Servo rightClawTurn;
    public final Servo rightClaw;
    public final Servo leftClaw;
    public final Servo door;
    public final Servo plane;

    public final DriveTrain driveTrain;
    public final Intake intake;

    private final OpMode opMode;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    public Robot(OpMode opMode) {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;

        driveTrain = new DriveTrain(hardwareMap);
        intake = new Intake(hardwareMap);

        climber  = hardwareMap.get(DcMotor.class, "climber");
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        leftClawTurn = hardwareMap.get(Servo.class, "leftClawTurn");
        rightClawTurn = hardwareMap.get(Servo.class, "rightClawTurn");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        door = hardwareMap.get(Servo.class, "door");
        plane = hardwareMap.get(Servo.class, "plane");
    }

    public void initialize() {
        telemetry.addData("Status", "Initialized");
    }

    public void start() {
        opMode.resetRuntime();
        leftLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        climber.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        climber.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightClawTurn.setDirection(Servo.Direction.REVERSE);
    }
}