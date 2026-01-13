package org.firstinspires.ftc.teamcode.testbed;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="TestBlueNear", group="Test")
public class TestAutoBlueNear extends LinearOpMode {
    TestRobotHardware robot = new TestRobotHardware(this);

    static final double DRIVE_SPEED = 0.5;     // Max driving speed for better distance accuracy.
    static final double TURN_SPEED = 0.3;     // Max turn speed to limit turn rate.
    static final double INTAKE_POWER = 1.0;
    static final double TRANSPORT_POWER = 0.8;
    static final double OUTTAKE_POWER = 0.35;
    static final double SHOOTING_TIME = 5.0;  // in Seconds

    @Override
    public void runOpMode() {

        robot.init();

        waitForStart();

        // Drive straight for 20 inches backward, heading at zero degree, and don't collect balls.
        autoDrive(false, -20, 0, false);

        // Shoot Balls
        autoShoot();

        // get the 2nd set of balls
        autoDrive(false, -10, 0, false);

        autoTurn(45); // change heading to 45 degrees.
        autoDrive(true, -17, 45, false);
        autoDrive(false, 28, 45, true);  // collect balls while driving
        autoDrive(false, -28, 45, false);
        autoDrive(true, 17, 45, false);

        autoTurn(0);  // change heading to 0 degrees.
        autoDrive(false, 10, 0, false);
        autoShoot();

        // park outside the launch zone and release gate
        autoTurn(-45);
        autoDrive(false, -25, -45, false);
        autoDrive(true, -25, -45, false);

    }

    public void autoDrive(boolean strafe, double distance, double heading, boolean collectBalls) {
        // Drive straight or strafe (if strafe is TRUE) at DRIVE_SPEED for "distance" (in inches), heading at "heading" degrees (-180 to +180),
        // and collect balls if collectBalls is true (if so, use INTAKE_POWER and TRANSPORT_POWER).
        robot.driveStraight(DRIVE_SPEED, strafe, distance, heading, collectBalls, INTAKE_POWER, TRANSPORT_POWER);
    }

    public void autoTurn(double heading) {
        // Turn at TURN_SPEED to heading degrees (-180 to +180)
        robot.turnToHeading(TURN_SPEED, heading);
    }

    public void autoShoot() {
        // Shoot Balls for SHOOTING_TIME seconds with OUTTAKE_POWER and TRANSPORT_POWER
        robot.shootBalls(OUTTAKE_POWER, TRANSPORT_POWER, SHOOTING_TIME);
    }
}
