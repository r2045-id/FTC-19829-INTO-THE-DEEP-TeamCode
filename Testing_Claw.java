package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.controllers.PIDCoefficients;
import org.firstinspires.ftc.teamcode.controllers.PIDFController;

import java.lang.reflect.Array;

@TeleOp
public class Testing_Claw extends OpMode {
    private DcMotorEx FrontLeft_Motor;
    private DcMotorEx FrontRight_Motor;
    private DcMotorEx RearLeft_Motor;
    private DcMotorEx RearRight_Motor;

    private DcMotorEx Extendo;
    private DcMotorEx Lifter;

    private Servo ArmRotator;
    private Servo ClawRotator;
    private Servo Claw;


    private boolean isY_gamepad2_Pressed = false;
    private boolean isDpadUp_pad1_Pressed = false;
    private boolean isDpadDown_pad1_Pressed = false;
    private boolean isY_pad1_Pressed = false;
    private boolean isA_pad1_Pressed = false;
    private boolean isB_pad1_Pressed = false;
    private boolean isB_pad2_Pressed = false;

    private boolean isA_pad2_Pressed = false;
    private double IntakePos = 0.05;
    private boolean isOuttakeInIdle = true;
    private boolean isSpecimenArmIdle = true;
    private boolean isSpecimenClawIdle = true;

    private double clawPos = 0;
    private double armPos = 0;


    private PIDCoefficients ExtendoCoeffs = new PIDCoefficients(0.01, 0, 0.0001);
    PIDFController ExtendoController = new PIDFController(ExtendoCoeffs);

    private PIDCoefficients LifterCoeffs = new PIDCoefficients(0.005, 0, 0.0001);
    PIDFController LifterController = new PIDFController(LifterCoeffs);

    private int extendo_currentSetpointIndex = 0;
    private int lifter_currentSetpointIndex = 0;

    private final int[] extendo_Setpoints = {0, 1000, 2120};
    private final int[] lifter_Setpoints = {5, 1000, 2180};

    private boolean extendoManualControl = false;
    private boolean lifterManualControl = false;
    private boolean isLifterAutomationFinished = false;

    private boolean initFinished = false;
    private double runtime = 0;
    private double lastSavedRuntime = 0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        FrontLeft_Motor  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        FrontRight_Motor = hardwareMap.get(DcMotorEx.class, "frontRight");
        RearRight_Motor  = hardwareMap.get(DcMotorEx.class, "rearRight");
        RearLeft_Motor = hardwareMap.get(DcMotorEx.class, "rearLeft");

        ArmRotator = hardwareMap.get(Servo.class, "armRotator");
        ClawRotator = hardwareMap.get(Servo.class, "clawRotator");
        Claw = hardwareMap.get(Servo.class, "claw");

        Extendo = hardwareMap.get(DcMotorEx.class, "extendo");
//        Lifter = hardwareMap.get(DcMotorEx.class, "lifter");

        Extendo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Extendo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        Lifter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        Lifter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        ArmRotator.setDirection(Servo.Direction.REVERSE);
//        Lifter.setDirection(DcMotorEx.Direction.REVERSE);
        Extendo.setDirection(DcMotorEx.Direction.FORWARD);

        FrontLeft_Motor.setDirection(DcMotorEx.Direction.FORWARD);
        FrontRight_Motor.setDirection(DcMotorEx.Direction.REVERSE);
        RearLeft_Motor.setDirection(DcMotorEx.Direction.FORWARD);
        RearRight_Motor.setDirection(DcMotorEx.Direction.REVERSE);
        ExtendoController.targetPosition = 0;
        LifterController.targetPosition = 0;
        telemetry.update();
    }

    @Override
    public void start() {
//        LifterController.targetPosition = (int) Array.get(lifter_Setpoints, 0);;
        ArmRotator.setPosition(0.25);
        Claw.setPosition(0);
        ClawRotator.setPosition(0);
        runtime = getRuntime();

    }

    @Override
    public void loop() {

        if (gamepad2.dpad_right && clawPos <= 1) {
            clawPos += 0.05;
            ClawRotator.setPosition(clawPos);
        }else if (gamepad2.dpad_left && clawPos >= 0) {
            clawPos -= 0.05;
            ClawRotator.setPosition(clawPos);
        }

        if (gamepad1.dpad_right && armPos <= 1) {
            armPos += 0.05;
            ArmRotator.setPosition(armPos);
        }else if (gamepad1.dpad_left && armPos >= 0) {
            armPos -= 0.05;
            ArmRotator.setPosition(armPos);
        }

        if (gamepad2.b) {
            if (!isB_pad2_Pressed) {
                isSpecimenClawIdle = !isSpecimenClawIdle;
                if (isSpecimenClawIdle) {
                    Claw.setPosition(0);
                }else {
                    Claw.setPosition(1);
                }
                isB_pad2_Pressed = true;
            }
        }else {
            isB_pad2_Pressed = false;
        }

        double slide = gamepad1.left_stick_x;
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double defspeed = gamepad1.left_trigger > 0 ? 1.0 : 0.6;

        FrontLeft_Motor.setPower(Range.clip(turn + drive + slide, -1.0, 1.0) * defspeed);
        FrontRight_Motor.setPower(Range.clip(-turn + drive - slide, -1.0, 1.0) * defspeed);
        RearRight_Motor.setPower(Range.clip(-turn + drive + slide, -1.0, 1.0) * defspeed);
        RearLeft_Motor.setPower(Range.clip(turn + drive - slide, -1.0, 1.0) * defspeed);

//        if ((Extendo.getCurrentPosition() <= 0 || (Extendo.getCurrentPosition() >0 && Extendo.getCurrentPosition() <= 210)) && isLifterAutomationFinished && Lifter.getCurrentPosition() < 500) {
//            isLifterAutomationFinished = false;
//        }else if (Extendo.getCurrentPosition() >= 210 || Lifter.getCurrentPosition() > 500) {
//            isLifterAutomationFinished = true;
//        }

        if (gamepad1.y || gamepad1.a) {
            extendoManualControl = false;
            if (gamepad1.y && !isY_pad1_Pressed && extendo_currentSetpointIndex != extendo_Setpoints.length-1) {
                extendo_currentSetpointIndex += 1;
                isY_pad1_Pressed = true;
            }else if (gamepad1.a && !isA_pad1_Pressed && extendo_currentSetpointIndex != 0) {
                extendo_currentSetpointIndex -= 1;
                isA_pad1_Pressed = true;
            }
            int desiredSetpoint = (int) Array.get(extendo_Setpoints, extendo_currentSetpointIndex);
            ExtendoController.targetPosition = desiredSetpoint;

        }else {
            if (!gamepad1.y && isY_pad1_Pressed) {
                isY_pad1_Pressed = false;
            }else if (!gamepad1.a && isA_pad1_Pressed) {
                isA_pad1_Pressed = false;
            }
        }

        if (gamepad1.dpad_up ) {
            Extendo.setPower( 0.75);
            extendoManualControl = true;
        }else if (gamepad1.dpad_down) {
            Extendo.setPower(-0.75);
            extendoManualControl = true;
        }else if (extendoManualControl) {
            Extendo.setPower(0);
        }

//        if (!extendoManualControl) {
//            double extendoDesiredPower = ExtendoController.update(Extendo.getCurrentPosition());
//            Extendo.setPower(extendoDesiredPower);
//        }

        // Lifter
//        if (gamepad2.dpad_up || gamepad2.dpad_down) {
//            lifterManualControl = false;
//            if (gamepad2.dpad_up && !isDpadUp_pad1_Pressed && lifter_currentSetpointIndex != lifter_Setpoints.length-1) {
//                lifter_currentSetpointIndex += 1;
//                isDpadUp_pad1_Pressed = true;
//            }else if (gamepad2.dpad_down && !isDpadDown_pad1_Pressed && lifter_currentSetpointIndex != 0) {
//                lifter_currentSetpointIndex -= 1;
//                isDpadDown_pad1_Pressed = true;
//            }
//
//            if (!isLifterAutomationFinished) {
//                extendoManualControl = false;
//                ArmRotator.setPosition(0.25);
//                ExtendoController.targetPosition = 210;
//            }
//            int desiredSetpoint = (int) Array.get(lifter_Setpoints, lifter_currentSetpointIndex);
//            LifterController.targetPosition = desiredSetpoint;
//        }else {
//            if (!gamepad2.dpad_up && isDpadUp_pad1_Pressed) {
//                isDpadUp_pad1_Pressed = false;
//            }else if (!gamepad2.dpad_down && isDpadDown_pad1_Pressed) {
//                isDpadDown_pad1_Pressed = false;
//            }
//        }
//
//        if (gamepad2.right_stick_y != 0 || lifterManualControl) {
//            lifterManualControl = true;
//            Lifter.setPower(-gamepad2.right_stick_y);
//        }
//
//        if (!lifterManualControl) {
//            if (isLifterAutomationFinished) {
//                double lifterDesiredPower = LifterController.update(Lifter.getCurrentPosition());
//                Lifter.setPower(lifterDesiredPower);
//            }
//        }



        if (Extendo.getCurrentPosition() == 0) {
//            ArmRotator.setPosition(0.05);
        }else {
            double nextIntakePos = IntakePos;
            if (Extendo.getVelocity() < -1000) {
                nextIntakePos = 0.05;
            }

            if (isA_pad1_Pressed) {
                nextIntakePos = 0.05;
            }

            if (gamepad1.b) {
                if (!isB_pad1_Pressed) {
                    if (IntakePos == 0.05 ) {
                        nextIntakePos = 0.91;
                    }else if (IntakePos == 0.91) {
                        nextIntakePos = 0.05;
                    }
                    isB_pad1_Pressed = true;
                }
            }else {
                isB_pad1_Pressed = false;
            }
            if (nextIntakePos != IntakePos) {
                IntakePos = nextIntakePos;
//                ArmRotator.setPosition(nextIntakePos);
            }
        }




        telemetry.addData("Claw Position ", clawPos);
        telemetry.addData("Arm Position ", armPos);

        telemetry.addData("Is Lifter Automation Finished", isLifterAutomationFinished);

        telemetry.addData("Gamepad Right Stick Y ", gamepad2.right_stick_y);
        telemetry.addData("Extendo Velocity", Extendo.getVelocity());
        telemetry.addData("Lifter Index", lifter_currentSetpointIndex);
        telemetry.addData("Lifter Setpoints", lifter_Setpoints);
        telemetry.addData("Dpad Left", gamepad1.dpad_left);

        telemetry.addData("Extendo Encoder ", Extendo.getCurrentPosition());

        telemetry.update();
    }
}
