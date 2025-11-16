/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.testbed;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * This OpMode illustrates how to use an external "hardware" class to modularize all the robot's sensors and actuators.
 * This approach is very efficient because the same hardware class can be used by all of your teleop and autonomous OpModes
 * without requiring many copy & paste operations.  Once you have defined and tested the hardware class with one OpMode,
 * it is instantly available to other OpModes.
 *
 * The real benefit of this approach is that as you tweak your robot hardware, you only need to make changes in ONE place (the Hardware Class).
 * So, to be effective you should put as much or your hardware setup and access code as possible in the hardware class.
 * Essentially anything you do with hardware in BOTH Teleop and Auto should likely go in the hardware class.
 *
 * The Hardware Class is created in a separate file, and then an "instance" of this class is created in each OpMode.
 * In order for the class to do typical OpMode things (like send telemetry data) it must be passed a reference to the
 * OpMode object when it's created, so it can access all core OpMode functions.  This is illustrated below.
 *
 * In this concept sample, the hardware class file is called RobotHardware.java and it must accompany this sample OpMode.
 * So, if you copy ConceptExternalHardwareClass.java into TeamCode (using Android Studio or OnBotJava) then RobotHardware.java
 * must also be copied to the same location (maintaining its name).
 *
 * For comparison purposes, this sample and its accompanying hardware class duplicates the functionality of the
 * RobotTelopPOV_Linear OpMode.  It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * View the RobotHardware.java class file for more details
 *
 *  Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 *  In OnBot Java, add a new OpMode, select this sample, and select TeleOp.
 *  Also add another new file named RobotHardware.java, select the sample with that name, and select Not an OpMode.
 */

@TeleOp(name="Drive0", group="Test")
// only intake and transport control
public class TestDrive0 extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    TestRobotHardware robot       = new TestRobotHardware(this);

    @Override
    public void runOpMode() {
        double forward, right, rotate, intakeP, outtakeP, transportP;
        boolean ballDetected = false;

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, or Strafe left/right, and the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            forward = -gamepad1.left_stick_y;
            right = gamepad1.left_stick_x;
            rotate  =  gamepad1.right_stick_x;

            // Combine drive and turn for blended motion. Use RobotHardware class
            robot.driveRobot(forward, right, rotate);

            // Color Sensor for Presence of Ball Close to the Shooting Flywheel
            String ballColor = robot.getColorSensor();
            if (!ballDetected)
                ballDetected = ballColor.equals("PURPLE") || ballColor.equals("GREEN");

            intakeP = gamepad1.left_trigger;
            outtakeP = gamepad1.right_trigger;
            transportP = Math.min(0.8, intakeP);

            robot.intakePower(intakeP);
            robot.transportPower(transportP);

            if (intakeP > 0.1 && outtakeP < 0.1) // during intake phase
                robot.outtakePower(-0.3); // prevent balls from getting ahead of the flywheel
            else // during outtake phase
                robot.outtakePower(0.8 * outtakeP);

            //reverse transport direction to put balls back in place
            if (gamepad1.left_bumper) {
                robot.outtakePower(-0.3);
                robot.transportPower(-0.3);
                robot.intakePower(0.5); // prevent balls from slipping out
            }

            // Send telemetry messages to explain controls and show robot status
            telemetry.addData("Drive forward/backward", "Left Stick (up/down)");
            telemetry.addData("Strafe left/right", "Left Stick (left/right)");
            telemetry.addData("Turn left/right", "Right Stick (left/right)");
            telemetry.addLine();
            telemetry.addData("Activate Intake and Transport", "Left Trigger");
            telemetry.addData("Reverse Transport", "Left Bumper");
            telemetry.addData("Activate Outtake", "Right Trigger");
            telemetry.addLine();
            telemetry.addData("Drive Power", "%.2f", forward);
            telemetry.addData("Strafe Power", "%.2f", right);
            telemetry.addData("Turn Power",  "%.2f", rotate);
            telemetry.addLine();
            telemetry.addData("Intake Power",  "%.2f", intakeP);
            telemetry.addData("Transport Power",  "%.2f", transportP);
            telemetry.addData("Outtake Power",  "%.2f", outtakeP);
            telemetry.addLine();
            telemetry.addData("Ball Detected", ballDetected);
            telemetry.addData("Color Sensor", ballColor);
            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            sleep(50);
        }
    }
}
