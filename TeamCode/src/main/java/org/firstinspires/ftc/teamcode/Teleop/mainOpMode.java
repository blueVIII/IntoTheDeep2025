package org.firstinspires.ftc.teamcode.Teleop;/* Copyright (c) 2017 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Mecanum Drive 24-25", group="Iterative Opmode")
public class mainOpMode extends OpMode
{
    //DONT DELETE
    private ElapsedTime runtime = new ElapsedTime();
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();
    private double Kg = 0.07;
    private boolean boxUp = false;
    //private PIDController pid = new PIDController(0.06, 0, 0);


    //main thingies
    private DcMotor frontLeft = null;
    private DcMotor rearLeft = null;
    private DcMotor frontRight = null;
    private DcMotor rearRight = null;
    private DcMotor liftMotor1 = null;
    private DcMotor liftMotor2 = null;
    private Servo liftClaw = null;
    private Servo liftClawRotate_Claw = null;
    private Servo liftClawRotate_Arm = null;
    private Servo liftClawExtender = null;


    //last year's thingies
    private DcMotor intakeMotor = null;
    private Servo rightLiftServo = null;
    private Servo leftLiftServo = null;
    private Servo door = null;
    private Servo plane = null;
    private Servo pixelHolder = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        //wheels
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        //lift
        liftMotor1 = hardwareMap.get(DcMotor.class, "liftMotor1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "liftMotor2");

        //things attached to lift
        liftClaw = hardwareMap.servo.get("liftClaw");
        liftClawRotate_Claw = hardwareMap.servo.get("liftClawRotate_Claw");
        liftClawRotate_Arm = hardwareMap.servo.get("liftClawRotate_Arm");
        liftClawExtender = hardwareMap.servo.get("liftClawExtender");

        /* 23-24 servos
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        rightLiftServo = hardwareMap.servo.get("rightLiftServo");
        leftLiftServo = hardwareMap.servo.get("leftLiftServo");
        door = hardwareMap.servo.get("door");
        pixelHolder = hardwareMap.servo.get("pixel");
        plane = hardwareMap.servo.get("plane");
        */


        //SETTINGS FOR MOTORS
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        liftMotor1.setDirection(DcMotor.Direction.FORWARD);
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //RUN_TO_POSITION

        liftMotor2.setDirection(DcMotor.Direction.REVERSE);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        //door.setPosition(0);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
    }

    // Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {

        //GAMEPAD 1 CONTROLS - Primary: Driving, Safety: Lift(Motors)
        //START DRIVING
        double frontLeftPower;
        double rearLeftPower;
        double frontRightPower;
        double rearRightPower;

        double y = gamepad1.left_stick_y * -1;
        double x = gamepad1.left_stick_x * 1.5;
        double pivot = gamepad1.right_stick_x;

        frontLeftPower = (pivot+y+x);
        rearLeftPower = (pivot+y-x);
        frontRightPower = (-pivot+y-x);
        rearRightPower = (-pivot+y+x);

        if(gamepad1.left_bumper) {
            frontLeft.setPower(frontLeftPower * 0.35);
            frontRight.setPower(frontRightPower * 0.35);
            rearLeft.setPower(rearLeftPower * 0.35);
            rearRight.setPower(rearRightPower * 0.35);
        }
        else {
            frontLeft.setPower(frontLeftPower * .85);
            frontRight.setPower(frontRightPower * .85);
            rearLeft.setPower(rearLeftPower * .85);
            rearRight.setPower(rearRightPower * .85);
        }
        //END DRIVING

        /* 23-24 had gp1 control intake, but for now, not planned
        if(gamepad1.right_bumper) {
            intakeMotor.setPower(-0.7);
        }
        else if (gamepad1.x) {
            intakeMotor.setPower(-0.7);
        }
        else {
            intakeMotor.setPower(0);
        }*/

        /*START GAMEPAD 1 SAFETY LIFT CONTROLS - not crucial
        int lift1Position = liftMotor1.getCurrentPosition();
        if(gamepad1.a) {
            lift1Position = liftMotor1.getCurrentPosition();
            if (lift1Position > 1000) {
                liftMotor1.setPower(-0.75);
                liftMotor2.setPower(-0.75);
            }
            else if (lift1Position <= 1000) {
                liftMotor1.setPower(-0.5);
                liftMotor2.setPower(-0.5);
            }
            else if (lift1Position <= 500) {
                liftMotor1.setPower(-0.3);
                liftMotor2.setPower(-0.3);
            }
        }
        if (gamepad1.y) {
            liftMotor1.setPower(1);
            liftMotor2.setPower(1);
        }
        else {
            liftMotor1.setPower(0);
            liftMotor2.setPower(0);
        }
        END GAMEPAD 1 SAFETY LIFT CONTROLS - not crucial*/





        //GAMEPAD 2 - Primary: Lift(Motors & Appendages)
        //START LIFT
        //targetPosition1 = liftMotor1.getCurrentPosition();
        //targetPosition2 = liftMotor2.getCurrentPosition();

        if (gamepad2.right_stick_y > 0.8) { //if stick is forward
            liftMotor1.setPower(0.8);
            liftMotor2.setPower(0.8);
        } else if (gamepad2.right_stick_y < -0.8) { //if stick is back
            liftMotor1.setPower(-0.6);
            liftMotor2.setPower(-0.6);
        } else { //idle, leave as 0 for now
            liftMotor1.setPower(0.2);
            liftMotor2.setPower(0.2);
        }

        /*    23-24 lift pos aware code
        if (gamepad2.right_stick_y > 0.8) {
//            liftMotor1.setPower(-1);
//            liftMotor2.setPower(-1);
            lift1Position = liftMotor1.getCurrentPosition();
            if (lift1Position < 200) {
                liftMotor1.setPower(-0.8);
                liftMotor2.setPower(-0.8);
            } else {
                liftMotor1.setPower(-1);
                liftMotor2.setPower(-1);
            }
//           else if (lift1Position <= 0) {
//                liftMotor1.setPower(0);
//                liftMotor2.setPower(0);
//            }
//            else if (lift1Position <= 50) {
//                liftMotor1.setPower(-0.35);
//                liftMotor2.setPower(-0.35);
//            }
//            else if (lift1Position <= 500) {
//                liftMotor1.setPower(-0.45);
//                liftMotor2.setPower(-0.45);
//            }
//            else if (lift1Position <= 1000) {
//                liftMotor1.setPower(-0.5);
//                liftMotor2.setPower(-0.5);
//            }
        } else if (gamepad2.right_stick_y < -0.8) {
            liftMotor1.setPower(1);
            liftMotor2.setPower(1);
        } else {
            liftMotor1.setPower(0.05);
            liftMotor2.setPower(0.05);
        }*/
        //END LIFT

        //START LIFT ATTACHMENTS CODE
        // - open claw/close claw, button (max, full close), DONE - A, B
        if (gamepad2.a) {
            liftClaw.setPosition(1); //idk positions
        } else if (gamepad2.b) {
            liftClaw.setPosition(0); //idk positions
        } //b opens, a closes

        // - rotate arm (trigger - left: rotate arm downward and hold, - right: rotate arm upward and hold)
        if (gamepad2.left_bumper) {
            liftClawRotate_Arm.setPosition(0.5); //idk positions
        } else if (gamepad2.right_bumper) {
            liftClawRotate_Arm.setPosition(1); //idk positions
        } //right rotates to floor, left rotates back up

        // - arm extender (left stick to extend/retract) WORKS, X Y
        if (gamepad2.x) {
            liftClawRotate_Claw.setPosition(1); //idk positions
        } else if (gamepad2.y) {
            liftClawRotate_Claw.setPosition(0); //idk positions
        }//x extends, y retracts

        // - rotate claw (claw itself): (bumpers, same setup as above)//DONE, WORKS FOR ROTATING, UP DOWN
        if (gamepad2.dpad_up) {
            liftClawExtender.setPosition(.6); //idk positions
        } else if (gamepad2.dpad_down) {
            liftClawExtender.setPosition(0); //idk positions
        } // up - rotate to ground, down - rotate upwards

        /* 23-24 lift closing/door servo code, not needed
        if (gamepad2.y) {
            rightLiftServo.setPosition(0.83);
//            leftLiftServo.setPosition(0);
        } else if (gamepad2.a) {
            rightLiftServo.setPosition(0.57);
//            leftLiftServo.setPosition(0.82);
        }

        if (gamepad2.left_bumper) {
            door.setPosition(0.7);
        } else if (gamepad2.right_bumper) {
            door.setPosition(0);
        }

        if (gamepad2.b) {
            pixelHolder.setPosition(1);
        } else if (gamepad2.x) {
            pixelHolder.setPosition(0);
        }

        if (gamepad2.dpad_up) {
            plane.setPosition(1);
        }
        else if (gamepad2.dpad_down) {
            plane.setPosition(0.7);
        }
        else if (gamepad2.dpad_left) {
            while (true) {
                liftMotor1.setPower(-1);
                liftMotor2.setPower(-1);
            }
        }
        else if (gamepad2.dpad_right) {
            pixelHolder.setPosition(0.4);
        }*/


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "frontLeft (%.2f), rearLeft (%.2f), frontRight (%.2f), rearRight (%.2f)", frontLeftPower, rearLeftPower, frontRightPower, rearRightPower);
        //telemetry.addData("Lift position", liftMotor1.getCurrentPosition());
        //telemetry.addData("Servo1", rightLiftServo.getPosition());
        //telemetry.addData("Servo2", leftLiftServo.getPosition());

        telemetry.update();
    }



//    public void liftToPosition(int target) {
//        int currentPosition1 = liftMotor1.getCurrentPosition();
//        int currentPosition2 = liftMotor2.getCurrentPosition();
//        while(Math.abs(currentPosition1 - target) > 6 && Math.abs(currentPosition2 - target) > 6) {
//            currentPosition1 = liftMotor1.getCurrentPosition();
//            currentPosition2 = liftMotor2.getCurrentPosition();
//            int targetPosition = target;
//            double power1 = returnPower(targetPosition, liftMotor1.getCurrentPosition());
//            double power2 = returnPower(targetPosition, liftMotor2.getCurrentPosition());
//            liftMotor1.setPower(power1);
//            liftMotor2.setPower(power2);
//            telemetry.addData("current position", currentPosition1);
//            telemetry.addData("targetPosition", targetPosition);
//            telemetry.update();
//        }
//    }



    public double returnPower(double reference, double state) {
        double error = reference - state;
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        double output = (error * 0.03) + (derivative * 0.0002) + 0.05;
        return output;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

