package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.controllers.PIDCoefficients;
import org.firstinspires.ftc.teamcode.controllers.PIDFController;

import java.lang.reflect.Array;

@Config
@TeleOp(group = "Tuning", name = "Lifter Tuner")
public class LifterTuner extends OpMode {
    private DcMotorEx Lifter;

    private int currentSetpointIndex = -1;
    private int currentSetpoint = -1;

    private boolean isAClicked = false;
    private final int[] setpoints = {1000, 2225, 0};


    public static PIDCoefficients coeffs = new PIDCoefficients(0.02, 0, 0.03);
    PIDFController pidController = new PIDFController(coeffs);

    @Override
    public void init() {
        Lifter = hardwareMap.get(DcMotorEx.class, "lifter");
        Lifter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Lifter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        Lifter.setDirection(DcMotorSimple.Direction.REVERSE);

        pidController.setOutputBounds(-1, 1);
    }

    @Override
    public void start() {
        telemetry.addLine("Press (A) to begin");
        telemetry.update();
    }

    public void loop() {
        if (!isAClicked && currentSetpointIndex == -1) {
            if (gamepad1.a) {
                currentSetpoint = (int) Array.get(setpoints, 0);
                pidController.targetPosition = currentSetpoint;
                currentSetpointIndex = 0;
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

        double power = pidController.update(Lifter.getCurrentPosition());
        Lifter.setPower(power);
        telemetry.addData("power",power);
        telemetry.addData("Setpoint",currentSetpoint );
        telemetry.addData("Positions",Lifter.getCurrentPosition() );

        telemetry.update();
    }
}
