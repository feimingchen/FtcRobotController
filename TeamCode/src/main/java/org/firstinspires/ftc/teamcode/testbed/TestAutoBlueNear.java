package org.firstinspires.ftc.teamcode.testbed;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="BlueNear", group="Test")
public class TestAutoBlueNear extends LinearOpMode {
    TestRobotHardware robot = new TestRobotHardware(this);

    static final double DRIVE_SPEED = 0.8;     // Max driving speed for better distance accuracy.
    static final double TURN_SPEED = 0.6;     // Max turn speed to limit turn rate.
    static final double INTAKE_POWER = 1.0;
    static final double TRANSPORT_POWER = 0.5;
    static final double OUTTAKE_POWER = 0.5;
    static final double SHOOTING_TIME = 5.0;  // in Seconds

    @Override
    public void runOpMode() {

        robot.init();

        waitForStart();

        // Drive straight for 20 inches backward, heading at zero degree, and don't collect balls.
        autoDrive(-20, 0, false);

        // Shoot Balls
        autoShoot();

        autoDrive(-35, 0, false);
        autoTurn(45);
        autoDrive(47, 45, true); // collect balls while driving
        autoDrive(-47, 45, false);
        autoTurn(0);
        autoDrive(35, 0, false);

        autoShoot();

        autoDrive(-30, -45, false); // park outside the launch zone
    }

    public void autoDrive(double distance, double heading, boolean collectBalls) {
        // Drive straight at DRIVE_SPEED for "distance" (in inches), heading at "heading" degrees (-180 to +180),
        // and collect balls if collectBalls is true (if so, use INTAKE_POWER and TRANSPORT_POWER).
        robot.driveStraight(DRIVE_SPEED, distance, heading, collectBalls, INTAKE_POWER, TRANSPORT_POWER);
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
