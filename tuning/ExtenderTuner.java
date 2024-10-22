package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.controllers.PIDCoefficients;
import org.firstinspires.ftc.teamcode.controllers.PIDFController;

import java.lang.reflect.Array;

@Config
@TeleOp(name = "Horizontal Extender Tuner", group = "Tuning")
public class ExtenderTuner extends OpMode {
    private DcMotorEx HorizontalExtender;

    private int currentSetpointIndex = -1;
    private int currentSetpoint = -1;

    private boolean isAClicked = false;
    private final int[] setpoints = {1000, 2140, 0};


    // 0.015, 0, 0.001
    public static PIDCoefficients coeffs = new PIDCoefficients(0,0,0);
    PIDFController pidController = new PIDFController(coeffs);

    @Override
    public void init() {
        HorizontalExtender = hardwareMap.get(DcMotorEx.class, "horizontalExtender");
        HorizontalExtender.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        HorizontalExtender.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        HorizontalExtender.setDirection(DcMotorSimple.Direction.REVERSE);

        pidController.setOutputBounds(-1, 1);
    }

    @Override
    public void start() {
        telemetry.addLine("Press (A) to begin");
        telemetry.update();
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
                if (currentSetpointIndex == setpoints.length-1) {
                    currentSetpointIndex = 0;
                }else {
                    currentSetpointIndex += 1;
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
