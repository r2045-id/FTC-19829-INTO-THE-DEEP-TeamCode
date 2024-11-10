package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controllers.PIDCoefficients;
import org.firstinspires.ftc.teamcode.controllers.PIDFController;

import java.lang.reflect.Array;

@Config
@TeleOp(name = "Horizontal Extender Tuner", group = "Tuning")
public class ExtenderTuner extends OpMode {
    private DcMotorEx HorizontalExtender;
    private Servo ArmPivot;

    private int currentSetpointIndex = -1;
    private int currentSetpoint = -1;

    private boolean isAClicked = false;
    private final int[] setpoints = {1000, 2140, 0};

    private boolean isIncreasing = true;

    // 0.015, 0, 0.001
    public static PIDCoefficients coeffs = new PIDCoefficients(0,0,0);
    PIDFController pidController = new PIDFController(coeffs);

    @Override
    public void init() {
        HorizontalExtender = hardwareMap.get(DcMotorEx.class, "horizontalExtender");
        HorizontalExtender.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        HorizontalExtender.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        HorizontalExtender.setDirection(DcMotorSimple.Direction.REVERSE);
        ArmPivot = hardwareMap.get(Servo.class, "armPivot");
        ArmPivot.setDirection(Servo.Direction.FORWARD);

        pidController.setOutputBounds(-1, 1);
    }

    @Override
    public void start() {
        telemetry.addLine("Press (A) to begin");
        telemetry.update();
        ArmPivot.setPosition(0.36);

    }

    @Override
    public void loop() {
        if (!isAClicked && currentSetpointIndex == -1) {
            if (gamepad1.a) {
                currentSetpoint = (int)Array.get(setpoints, 0);
                pidController.targetPosition = currentSetpoint;
                currentSetpointIndex = 0;
                isAClicked = true;
            }
            return;
        }

        if (gamepad1.a) {
            if (!isAClicked) {
                if (isIncreasing && currentSetpointIndex == setpoints.length - 1) {
                    isIncreasing = false;
                }else if (!isIncreasing && currentSetpointIndex == 0) {
                    isIncreasing = true;
                }

                if (isIncreasing && currentSetpointIndex != setpoints.length - 1) {
                    currentSetpointIndex += 1;
                }else if (!isIncreasing && currentSetpointIndex != 0) {
                    currentSetpointIndex -= 1;
                }

                currentSetpoint = (int)Array.get(setpoints, currentSetpointIndex);
                pidController.targetPosition = currentSetpoint;
                isAClicked = true;
            }
        }else {
            isAClicked = false;
        }

        double power = pidController.update(HorizontalExtender.getCurrentPosition());
        HorizontalExtender.setPower(power);
        telemetry.addData("power",power);
        telemetry.addData("Setpoint",currentSetpoint );
        telemetry.addData("Positions",HorizontalExtender.getCurrentPosition() );

        telemetry.update();
    }
}
