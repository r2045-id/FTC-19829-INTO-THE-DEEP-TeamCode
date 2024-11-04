package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


@TeleOp
public class DrivebaseTest extends OpMode {
    private DcMotor FrontLeft_Motor;
    private DcMotor FrontRight_Motor;
    private DcMotor RearLeft_Motor;
    private DcMotor RearRight_Motor;

    private OverflowEncoder xAxisEncoder;
    private OverflowEncoder yAxisEncoder;

//    private

    @Override
    public void init() {
        FrontLeft_Motor  = hardwareMap.get(DcMotor.class, "frontLeft");
        FrontRight_Motor = hardwareMap.get(DcMotor.class, "frontRight");
        RearRight_Motor  = hardwareMap.get(DcMotor.class, "rearRight");
        RearLeft_Motor = hardwareMap.get(DcMotor.class, "rearLeft");

        FrontLeft_Motor.setDirection(DcMotor.Direction.FORWARD);
        FrontRight_Motor.setDirection(DcMotor.Direction.REVERSE);
        RearLeft_Motor.setDirection(DcMotor.Direction.FORWARD);
        RearRight_Motor.setDirection(DcMotor.Direction.REVERSE);

        FrontLeft_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearLeft_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearRight_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearLeft_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearRight_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        xAxisEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rearRight")));
        yAxisEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frontLeft")));

        yAxisEncoder.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {
        double slide = gamepad1.left_stick_x;
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double defspeed = gamepad1.left_trigger > 0 ? 1.0 : 0.6;

        FrontLeft_Motor.setPower(Range.clip(turn + drive + slide, -1.0, 1.0) * defspeed);
        FrontRight_Motor.setPower(Range.clip(-turn + drive - slide, -1.0, 1.0) * defspeed);
        RearRight_Motor.setPower(Range.clip(-turn + drive + slide, -1.0, 1.0) * defspeed);
        RearLeft_Motor.setPower(Range.clip(turn + drive - slide, -1.0, 1.0) * defspeed);
        PositionVelocityPair xAxisPosVel = xAxisEncoder.getPositionAndVelocity();
        PositionVelocityPair yAxisPosVel = yAxisEncoder.getPositionAndVelocity();


        telemetry.addData("X Pos", xAxisPosVel.position);
        telemetry.addData("X Raw Pos", xAxisPosVel.position);
        telemetry.addData("Y Pos", yAxisPosVel.position);
        telemetry.addData("Y Raw Pos", yAxisPosVel.position);

        telemetry.addData("rear Right Pos", RearRight_Motor.getCurrentPosition());
        telemetry.addData("front Right Raw Pos", FrontRight_Motor.getCurrentPosition());
        telemetry.addData("rear Left Pos", RearLeft_Motor.getCurrentPosition());
        telemetry.addData("front Left Raw Pos", FrontLeft_Motor.getCurrentPosition());

        telemetry.update();
    }
}
