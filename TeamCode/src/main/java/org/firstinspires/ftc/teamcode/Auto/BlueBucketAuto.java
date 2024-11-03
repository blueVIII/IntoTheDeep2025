package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "BlueBucketAuto", group = "Autonomous")
public class BlueBucketAuto extends LinearOpMode {

    /*
    public class Lift {
        private DcMotorEx lift1;
        private DcMotorEx lift2;

        public Lift(HardwareMap hardwareMap) {
            lift1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
            lift2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");
            lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift1.setDirection(DcMotorSimple.Direction.FORWARD);
            lift2.setDirection(DcMotorSimple.Direction.FORWARD);

        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    System.out.println("Going up!");
                    lift1.setPower(0.8);
                    lift2.setPower(-0.8);

                    initialized = true;
                }

                double pos1 = lift1.getCurrentPosition();
                double pos2 = lift2.getCurrentPosition();

                packet.put("liftPos1", pos1);
                packet.put("liftPos2", pos2);

                if (pos1 < 3000.0 & pos2 < 3000.0) {
                    return true;
                } else {
                    lift1.setPower(0);
                    lift2.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift1.setPower(-0.8);
                    lift2.setPower(0.8);
                    initialized = true;
                }

                double pos1 = lift1.getCurrentPosition();
                double pos2 = lift2.getCurrentPosition();

                packet.put("liftPos1", pos1);
                packet.put("liftPos2", pos2);

                if (pos1 > 100.0 & pos2 > 100.0) {
                    return true;
                } else {
                    lift1.setPower(0);
                    lift2.setPower(0);

                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }
    */


    /*
    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.55);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(1.0);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }
    */

    //Close the Claw: The robot starts by closing its claw to hold onto something (probably a game element)
    //Wait for Start: The robot waits for the match to begin and checks its starting position using vision.
    //Move to a Location: Based on the starting position detected by the vision system, the robot selects a path and drives to a specific location on the field.
    //Lift the Slide: The robot raises its lift (probably to place an object on a higher level).
    //Open the Claw: After lifting, the robot opens the claw to release the object it was holding.
    //Lower the Slide: Once the object is placed, the robot lowers the lift back down.
    //Move to a Final Location: Finally, the robot drives to another location to complete the autonomous routine, getting ready for the next phase of the match.

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, 61.7, Math.toRadians(90)));
        //Claw claw = new Claw(hardwareMap);
        //Lift lift = new Lift(hardwareMap); //lifts work,

        Action driveToSub, driveToFirstSample;

        driveToSub = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(48, 12))
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(0) //set to wait for length of lift raise, or maybe leave at 0 - likely latter
                .build();
        /*driveToFirstSample = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(0) //set to wait for claw close, or maybe leave at 0 - likely latter
                .build();*/


        // actions that need to happen on init; for instance, a claw tightening.
        //Actions.runBlocking(claw.closeClaw());

        while (!isStarted() && isStopRequested()) telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            driveToSub
                            //lift.liftUp(),
                            //lift.liftDown()
                    )
            );
        }
    }
}
