package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Test: Motor", group = "Test")
public class TestMotor extends LinearOpMode {

    private DcMotor intake;
    private DcMotor outtake;

    private CRServo FrontRight;
    private CRServo FrontLeft;
    private CRServo BackRight;
    private CRServo BackLeft;

    @Override
    public void runOpMode() {
        outtake = hardwareMap.get(DcMotor.class, "FrontLeft");
        outtake.setDirection(DcMotorSimple.Direction.FORWARD);

        intake = hardwareMap.get(DcMotor.class, "FrontRight");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontLeft = hardwareMap.get(CRServo.class, "FR");
        FrontRight = hardwareMap.get(CRServo.class, "FL");
        BackLeft = hardwareMap.get(CRServo.class, "BR");
        BackRight = hardwareMap.get(CRServo.class, "BL");

        FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            intake.setPower(0.7);

            // spin servos at full power
            FrontLeft.setPower(1.0);
            FrontRight.setPower(1.0);
            BackLeft.setPower(1.0);
            BackRight.setPower(1.0);
        }
    }
}
