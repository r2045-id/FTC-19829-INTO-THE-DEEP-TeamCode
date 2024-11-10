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

    private Servo ArmPivot;
    private Servo ClawRotator;
    private Servo Claw;
    private Servo Speciment;
    private Servo Bucket;
    private Servo BucketRotator;

    private boolean isDpadUp_pad1_Pressed = false;
    private boolean isDpadDown_pad1_Pressed = false;
    private boolean isY_pad1_Pressed = false;
    private boolean isA_pad1_Pressed = false;
    private boolean isB_pad1_Pressed = false;
    private boolean isB_pad2_Pressed = false;
    private boolean isA_pad2_Pressed = false;
    private boolean isX_pad2_Pressed = false;

    private boolean isY_pad2_Pressed = false;
    private boolean isLeftBumper_pad1_Pressed = false;

    private double IntakePos = 0.05;

    private boolean isOuttakeInIdle = true;
    private boolean isSpecimenArmIdle = true;
    private boolean isClawIdle = true;
    private boolean isBucketIdle = true;
    private boolean isBucketRotatorIdle = true;
    private boolean isSpecimenClawIdle = false;
    private boolean isClawDirClockwise = true;
    private boolean isArmPivotDirClockwise = true;

    private double clawRotatorPlacementPos = 0.355;
    private double clawRotatorDefaultPos = 0;
    private double clawClosedPos = 0.75;
    private double clawOpenPos = 0;
    private double specimentClosedPos = 0.345;
    private double specimentOpenPos  = 1;
    private double bucketIdlePos  = 0.1425;

    private final double[] armPivot_setpoints = { 0.32, 0.8, 0.935};
    private int armPivot_setpointIndex = 0;
    private double armIdlePos = 0.32;

    private double maxArmPivotPos  = 0.4;
    private double armPivotIdle  = 0.32;

    private double maxGripSpecimentPos  = 0.345;
    private double clawRotatorPos = 0;
    private double armPos = 0;
    private double bucketPos = 0;

    private PIDCoefficients ExtendoCoeffs = new PIDCoefficients(0.01, 0, 0.0001);
    PIDFController ExtendoController = new PIDFController(ExtendoCoeffs);

    private PIDCoefficients LifterCoeffs = new PIDCoefficients(0.005, 0, 0.0001);
    PIDFController LifterController = new PIDFController(LifterCoeffs);

    private int extendo_currentSetpointIndex = 0;
    private int lifter_currentSetpointIndex = 0;

    private final int[] extendo_Setpoints = {0, 1000,  2850};
    private final int[] lifter_Setpoints = {5, 1000, 2015};

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

        ArmPivot = hardwareMap.get(Servo.class, "armPivot");
        ClawRotator = hardwareMap.get(Servo.class, "clawRotator");
        Claw = hardwareMap.get(Servo.class, "claw");
        Speciment = hardwareMap.get(Servo.class, "speciment");
        Bucket = hardwareMap.get(Servo.class, "bucket");
        BucketRotator = hardwareMap.get(Servo.class, "bucketRotator");

        Extendo = hardwareMap.get(DcMotorEx.class, "extendo");
        Lifter = hardwareMap.get(DcMotorEx.class, "lifter");

        Extendo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Extendo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        ArmPivot.setDirection(Servo.Direction.FORWARD);
        Extendo.setDirection(DcMotorEx.Direction.REVERSE);
        Lifter.setDirection(DcMotorEx.Direction.REVERSE);

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
        ArmPivot.setPosition(0.32);
        Claw.setPosition(clawOpenPos);
        ClawRotator.setPosition(0.35);
        runtime = getRuntime();
        bucketPos = bucketIdlePos;
        Bucket.setPosition(bucketIdlePos);
        Speciment.setPosition(specimentOpenPos);
        BucketRotator.setPosition(0);
    }

    @Override
    public void loop() {

        if (gamepad1.left_bumper) {
            if (!isLeftBumper_pad1_Pressed) {
                if (isArmPivotDirClockwise && armPivot_setpointIndex == armPivot_setpoints.length - 1) {
                    isArmPivotDirClockwise = false;
                }else if ((isClawIdle && !isArmPivotDirClockwise && armPivot_setpointIndex == 1) || (!isArmPivotDirClockwise && armPivot_setpointIndex == 0)) {
                    isArmPivotDirClockwise = true;
                }

                if (isArmPivotDirClockwise && armPivot_setpointIndex != armPivot_setpoints.length ) {
                    if (armPivot_setpointIndex + 1 == armPivot_setpoints.length -1) {
                        Claw.setPosition(clawClosedPos);
                        isClawIdle =  false;
                    }
                    armPivot_setpointIndex += 1;

                }else if (!isArmPivotDirClockwise && armPivot_setpointIndex != 0) {
                    if ((!isClawIdle ) || (isClawIdle && armPivot_setpointIndex - 1 != 0)) {
                        armPivot_setpointIndex -= 1;
                    }

                    if (!isClawIdle) {
                        clawRotatorPos = 0.32;
                        ClawRotator.setPosition(clawRotatorPos);
                    }
                }
                double desiredSetpoint = (double) Array.get(armPivot_setpoints, armPivot_setpointIndex);
                armPos = desiredSetpoint;
                ArmPivot.setPosition(armPos);
                isLeftBumper_pad1_Pressed = true;
            }
        }else {
            isLeftBumper_pad1_Pressed = false;
        }

        telemetry.addData("armPos", armPos);

        if (gamepad1.x) {
            armPos = armIdlePos;
            ArmPivot.setPosition(armPos);
        }

        if (gamepad1.b) {
            if (!isB_pad1_Pressed) {
                isClawIdle = !isClawIdle;
                if (isClawIdle) {
                    Claw.setPosition(clawOpenPos);
                }else {
                    Claw.setPosition(clawClosedPos);
                }
                isB_pad1_Pressed = true;
            }
        }else {
            isB_pad1_Pressed = false;
        }

        if (gamepad1.right_bumper) {
            if (isClawDirClockwise && clawRotatorPos <= 0.685) {
                clawRotatorPos += 0.025;
            }else if (!isClawDirClockwise && clawRotatorPos > 0) {
                clawRotatorPos -= 0.025;
            }else {
                if (clawRotatorPos >= 0.685) {
                    isClawDirClockwise = false;
                }else if (clawRotatorPos <= 0) {
                    isClawDirClockwise = true;
                }
            }
            ClawRotator.setPosition(clawRotatorPos);
        }

        if (gamepad2.a) {
            if (!isA_pad2_Pressed) {
                isBucketIdle = !isBucketIdle;
                if (isBucketIdle) {
                    bucketPos = bucketIdlePos;
                    Bucket.setPosition(bucketIdlePos);
                    isBucketRotatorIdle = true;
                    BucketRotator.setPosition(0);
                }else if ( Lifter.getCurrentPosition() > 500){
                    bucketPos = 0.675;
                    Bucket.setPosition(0.675);
                    isBucketRotatorIdle = true;
                    BucketRotator.setPosition(0);
                }
                isA_pad2_Pressed = true;
            }
        }else {
            isA_pad2_Pressed = false;
        }

        if (gamepad2.x) {
            if (!isX_pad2_Pressed) {
                isBucketRotatorIdle = !isBucketRotatorIdle;

                if (isBucketRotatorIdle) {
                    BucketRotator.setPosition(0);
                }else {
                    BucketRotator.setPosition(0.665);
                }
                isX_pad2_Pressed = true;
            }
        }else {
            isX_pad2_Pressed = false;
        }

        if (gamepad2.b) {
            if (!isB_pad2_Pressed) {
                isSpecimenClawIdle = !isSpecimenClawIdle;
                if (isSpecimenClawIdle) {
                    Speciment.setPosition(specimentOpenPos);
                }else {
                    Speciment.setPosition(specimentClosedPos);
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
            if (!gamepad1.a && isY_pad1_Pressed) {
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

        if (!extendoManualControl) {
            double extendoDesiredPower = ExtendoController.update(Extendo.getCurrentPosition());
            Extendo.setPower(extendoDesiredPower);
        }

        if (Extendo.getCurrentPosition() == 0) {
//            ArmRotator.setPosition(0.05);
        }else {
            double nextArmPos = armPos;
            if (Extendo.getVelocity() < -1800) {
                nextArmPos = armIdlePos;
            }

            if (nextArmPos != armPos) {
                armPos = nextArmPos;
                ArmPivot.setPosition(nextArmPos);
            }
        }


        // Lifter
        if (gamepad2.dpad_up || gamepad2.dpad_down) {
            lifterManualControl = false;
            if (gamepad2.dpad_up && !isDpadUp_pad1_Pressed && lifter_currentSetpointIndex != lifter_Setpoints.length-1) {
                if (lifter_currentSetpointIndex == 0) {
                    bucketPos = 0.1;
                    Bucket.setPosition(0.1);
                    armPos = 0.55;
                    ArmPivot.setPosition(armPos);
                    BucketRotator.setPosition(0.665);
                }
                lifter_currentSetpointIndex += 1;
                isDpadUp_pad1_Pressed = true;
            }else if (gamepad2.dpad_down && !isDpadDown_pad1_Pressed && lifter_currentSetpointIndex != 0) {
                lifter_currentSetpointIndex -= 1;
                isDpadDown_pad1_Pressed = true;
            }

            int desiredSetpoint = (int) Array.get(lifter_Setpoints, lifter_currentSetpointIndex);
            LifterController.targetPosition = desiredSetpoint;
        }else {
            if (!gamepad2.dpad_up && isDpadUp_pad1_Pressed) {
                isDpadUp_pad1_Pressed = false;
            }else if (!gamepad2.dpad_down && isDpadDown_pad1_Pressed) {
                isDpadDown_pad1_Pressed = false;
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

        if (Lifter.getCurrentPosition() < -1000) {
            bucketPos = 0.1425;
            Bucket.setPosition(0.1425);

        }



        telemetry.addData("Lifter Manual Control ", lifterManualControl);


        telemetry.addData("Claw Position ", clawRotatorPos);
        telemetry.addData("Arm Position ", armPos);

        telemetry.addData("Gamepad Right Stick Y ", gamepad2.right_stick_y);
        telemetry.addData("Extendo Velocity", Extendo.getVelocity());
        telemetry.addData("Lifter Index", lifter_currentSetpointIndex);
        telemetry.addData("Lifter Setpoints", lifter_Setpoints);
        telemetry.addData("Lifter Pos", Lifter.getCurrentPosition());
        telemetry.addData("Lifter Vel", Lifter.getVelocity());

        telemetry.addData("Dpad Left", gamepad1.dpad_left);

        telemetry.addData("Extendo Encoder ", Extendo.getCurrentPosition());

        telemetry.update();
    }
}
