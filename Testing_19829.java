package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controllers.PIDCoefficients;
import org.firstinspires.ftc.teamcode.controllers.PIDFController;

import java.lang.reflect.Array;

//@Config
@TeleOp(group = "A")
public class Testing_19829 extends OpMode {
    private DcMotorEx FrontLeft_Motor;
    private DcMotorEx FrontRight_Motor;
    private DcMotorEx RearLeft_Motor;
    private DcMotorEx RearRight_Motor;

    private DcMotorEx Extendo;
    private DcMotorEx Lifter;

    private CRServo Intake;
    private Servo IntakeRotator;
    private Servo Outtake;

    private boolean isY_gamepad2_Pressed = false;
    private boolean isDpadUp_pad1_Pressed = false;
    private boolean isDpadDown_pad1_Pressed = false;
    private boolean isDpadRight_pad1_Pressed = false;
    private boolean isDpadLeft_pad1_Pressed = false;

    private PIDCoefficients ExtendoCoeffs = new PIDCoefficients(0.015, 0, 0.001);
    PIDFController ExtendoController = new PIDFController(ExtendoCoeffs);

    private PIDCoefficients LifterCoeffs = new PIDCoefficients(0.02, 0, 0.03);
    PIDFController LifterController = new PIDFController(LifterCoeffs);

    private int extendo_currentSetpointIndex = 0;
    private int lifter_currentSetpointIndex = 0;

    private final int[] extendo_Setpoints = {0, 1000, 2160};
    private final int[] lifter_Setpoints = {5, 1000, 2225};

    private boolean extendoManualControl = false;
    private boolean lifterManualControl = false;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        FrontLeft_Motor  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        FrontRight_Motor = hardwareMap.get(DcMotorEx.class, "frontRight");
        RearRight_Motor  = hardwareMap.get(DcMotorEx.class, "rearRight");
        RearLeft_Motor = hardwareMap.get(DcMotorEx.class, "rearLeft");

        Intake = hardwareMap.get(CRServo.class, "intake");
        IntakeRotator = hardwareMap.get(Servo.class, "intakeRotator");
        Outtake = hardwareMap.get(Servo.class, "outtake");

        Extendo = hardwareMap.get(DcMotorEx.class, "horizontalExtender");
        Lifter = hardwareMap.get(DcMotorEx.class, "lifter");

        Extendo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Extendo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        Lifter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Lifter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        IntakeRotator.setDirection(Servo.Direction.FORWARD);
        Outtake.setDirection(Servo.Direction.FORWARD);

        Lifter.setDirection(DcMotorEx.Direction.REVERSE);
        Extendo.setDirection(DcMotorEx.Direction.REVERSE);

        FrontLeft_Motor.setDirection(DcMotorEx.Direction.FORWARD);
        FrontRight_Motor.setDirection(DcMotorEx.Direction.REVERSE);
        RearLeft_Motor.setDirection(DcMotorEx.Direction.FORWARD);
        RearRight_Motor.setDirection(DcMotorEx.Direction.REVERSE);
        ExtendoController.targetPosition = 0;
        telemetry.update();
    }

    @Override
    public void start() {
//        LifterController.targetPosition = (int) Array.get(lifter_Setpoints, 0);;
    }

    @Override
    public void loop() {

        if (gamepad1.x) {
            Outtake.setPosition(0);
        }else if (gamepad1.a && Lifter.getCurrentPosition() > 300) {
            Outtake.setPosition(0.6);
        }


        if (gamepad1.dpad_up || gamepad1.dpad_down) {
            extendoManualControl = false;

            if (gamepad1.dpad_up && !isDpadUp_pad1_Pressed && extendo_currentSetpointIndex != extendo_Setpoints.length-1) {

                extendo_currentSetpointIndex += 1;
                isDpadUp_pad1_Pressed = true;

            }else if (gamepad1.dpad_down && !isDpadDown_pad1_Pressed && extendo_currentSetpointIndex != 0) {
                extendo_currentSetpointIndex -= 1;
                isDpadUp_pad1_Pressed = true;
            }
            int desiredSetpoint = (int) Array.get(extendo_Setpoints, extendo_currentSetpointIndex);
            ExtendoController.targetPosition = desiredSetpoint;
        }else {
            if (!gamepad1.dpad_up && isDpadUp_pad1_Pressed) {
                isDpadUp_pad1_Pressed = false;
            }else if (!gamepad1.dpad_down && isDpadDown_pad1_Pressed) {
                isDpadDown_pad1_Pressed = false;
            }
        }

        if (gamepad1.left_trigger != 0) {
            Extendo.setPower(1);
            extendoManualControl = true;
        }else if (gamepad1.right_trigger != 0) {
            Extendo.setPower(-1);
            extendoManualControl = true;
        }else if (extendoManualControl) {
            Extendo.setPower(0);
        }

        if (!extendoManualControl) {
            double extendoDesiredPower = ExtendoController.update(Extendo.getCurrentPosition());
            Extendo.setPower(extendoDesiredPower);
        }

        // Lifter
        if (gamepad1.dpad_right || gamepad1.dpad_left) {
            lifterManualControl = false;

            if (gamepad1.dpad_right && !isDpadRight_pad1_Pressed && lifter_currentSetpointIndex != lifter_Setpoints.length-1) {

                lifter_currentSetpointIndex += 1;
                isDpadRight_pad1_Pressed = true;

            }else if (gamepad2.dpad_left && !isDpadLeft_pad1_Pressed && lifter_currentSetpointIndex != 0) {
                lifter_currentSetpointIndex -= 1;
                isDpadLeft_pad1_Pressed = true;
            }
            int desiredSetpoint = (int) Array.get(lifter_Setpoints, extendo_currentSetpointIndex);
            LifterController.targetPosition = desiredSetpoint;
        }else {
            if (!gamepad1.dpad_right && isDpadRight_pad1_Pressed) {
                isDpadRight_pad1_Pressed = false;
            }else if (!gamepad1.dpad_left && isDpadLeft_pad1_Pressed) {
                isDpadLeft_pad1_Pressed = false;
            }
        }

        if (gamepad2.right_stick_y != 0 || lifterManualControl) {
            lifterManualControl = true;
            Lifter.setPower(-gamepad2.right_stick_y);
        }

        if (!lifterManualControl) {
            double lifterDesiredPower = LifterController.update(Lifter.getCurrentPosition());
            Lifter.setPower(lifterDesiredPower);
        }

        if (Extendo.getCurrentPosition() == 0) {
            IntakeRotator.setPosition(0.05);
        }else {
            double IntakePos = 0.05;
            if (Extendo.getCurrentPosition() > 500) {
                IntakePos = 0.9;
            }
            if (gamepad1.y) {
                IntakePos = 0.9;
            }else if (gamepad1.b) {
                IntakePos = 0.05;
            }
            IntakeRotator.setPosition(IntakePos);
        }

        if (gamepad1.right_bumper) {
            Intake.setPower(1);
        }else if (gamepad1.left_bumper){
            Intake.setPower(-1);
        }else {
            Intake.setPower(0);
        }



        if (gamepad2.y) {
            if (isY_gamepad2_Pressed == false) {

                isY_gamepad2_Pressed = true;
            }
        }else {
            isY_gamepad2_Pressed = false;
        }

        telemetry.addData("Gamepad Right Stick Y ", gamepad2.right_stick_y);

        telemetry.addData("Lifter Encoder ", Lifter.getCurrentPosition());
        telemetry.addData("Extendo Encoder ", Extendo.getCurrentPosition());

        telemetry.update();
    }
}
