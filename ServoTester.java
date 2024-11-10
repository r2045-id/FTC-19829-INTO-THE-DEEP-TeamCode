package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTester extends OpMode {
    private Servo claw;
    private Servo clawRotator;
    private Servo armPivot;
    private Servo bucketRotator;
    private Servo bucket;
    private Servo speciment;

    private double clawPos = 0;
    private double clawRotatorPos = 0;
    private  double armPivotPos = 0.57;
    private  double bucketRotatorPos = 0;
    private  double bucketPos = 0;
    private  double specimentPos = 0;

    @Override
    public void init() {
        claw = hardwareMap.get(Servo.class, "claw");
        clawRotator = hardwareMap.get(Servo.class, "clawRotator");
        armPivot = hardwareMap.get(Servo.class, "armPivot");
        bucketRotator = hardwareMap.get(Servo.class, "bucketRotator");
        bucket = hardwareMap.get(Servo.class, "bucket");
        speciment = hardwareMap.get(Servo.class, "speciment");

    }

    @Override
    public void start() {
        claw.setPosition(clawPos);
        clawRotator.setPosition(clawRotatorPos);
        armPivot.setPosition(0.57);
        bucketRotator.setPosition(bucketRotatorPos);
        bucket.setPosition(bucketPos);

        speciment.setPosition(specimentPos);
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_up && clawPos <= 1) {
            clawPos += 0.005;
        }else if (gamepad1.dpad_down && clawPos >= 0) {
            clawPos -= 0.005;
        }

        if (gamepad1.dpad_right && clawRotatorPos <= 1) {
            clawRotatorPos += 0.005;
        }else if (gamepad1.dpad_left && clawRotatorPos >= 0) {
            clawRotatorPos -= 0.005;
        }

        if (gamepad2.dpad_right && armPivotPos <= 1) {
            armPivotPos += 0.005;
        }else if (gamepad2.dpad_left && armPivotPos >= 0) {
            armPivotPos -= 0.005;
        }

        if (gamepad2.dpad_up && bucketRotatorPos <= 1) {
            bucketRotatorPos += 0.005;
        }else if (gamepad2.dpad_down && bucketRotatorPos >= 0) {
            bucketRotatorPos -= 0.005;
        }


        if (gamepad2.y && bucketPos <= 1) {
            bucketPos += 0.005;
        }else if (gamepad2.a && bucketPos >= 0) {
            bucketPos -= 0.005;
        }

        if (gamepad2.x && specimentPos <= 1) {
            specimentPos += 0.005;
        }else if (gamepad2.b && specimentPos >= 0) {
            specimentPos -= 0.005;
        }

        claw.setPosition(clawPos);
        clawRotator.setPosition(clawRotatorPos);
        armPivot.setPosition(armPivotPos);
        bucket.setPosition(bucketPos);
        bucketRotator.setPosition(bucketRotatorPos);
        speciment.setPosition(specimentPos);

        telemetry.addData("clawPos", clawPos);
        telemetry.addData("clawRotator Pos", clawRotatorPos);
        telemetry.addData("armPivot Pos", armPivotPos);

        telemetry.addData("bucketRotator Pos", bucketRotatorPos);
        telemetry.addData("bucket Pos", bucketPos);
        telemetry.addData("speciment Pos", specimentPos);

        telemetry.update();
    }

}
