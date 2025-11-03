package org.firstinspires.ftc.teamcode.testbed;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Test: Motor", group = "Test")

public class TestMotor extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor m0 = hardwareMap.get(DcMotor.class, "m0");
        m0.setDirection(DcMotorSimple.Direction.FORWARD);

        CRServo s0 = hardwareMap.get(CRServo.class, "s0");

        waitForStart();

        double mPower, sPower;

        while (opModeIsActive()) {
            mPower = -this.gamepad1.left_stick_y;
            m0.setPower(mPower);

            sPower = this.gamepad1.right_stick_y;
            s0.setPower(sPower);

            telemetry.addData("Motor Power", m0.getPower());
            telemetry.addData("Servo Power", s0.getPower());
            telemetry.update();
        }
    }
}
