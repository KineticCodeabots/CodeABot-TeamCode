/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOpMode")
public class TeleOpMode extends LinearOpMode {
    // Constants
    final private double DRIVE_SMOOTHING = 2;
    final private double MAX_TURN = 0.5;
    final private double ARM_MAX_POWER = 0.4;
    final private double ARM_DOWN_MAX_POWER = 0.1;
    final private double MAX_CRAWL_SPEED = 0.3;
    final private double MAX_PRECISE_SPEED = 0.2;

    // Gamepads to determine state changes.
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    private ElapsedTime runtime = new ElapsedTime();
    private Robot robot = new Robot(this);

    private boolean crawlingMode = false;
    private boolean preciseMode = false;

    @Override
    public void runOpMode() {
        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            // Update gamepads
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            updateDrive();
            updateArm();

            // Send telemetry
            telemetry.addLine("Driver (Gamepad 1):");
            telemetry.addData("Drive (Left Stick)", "left (%.2f), right (%.2f)", robot.leftSpeed, robot.rightSpeed);
            telemetry.addData("Crawling (Left Bumber): Precise (Right Bumber)", "%b : %b", crawlingMode, preciseMode);

            telemetry.addLine("\nOperator (Gamepad 2):");
            telemetry.addData("Arm (Left Stick)", "power (%.2f), position (%d)", robot.armMotor.getPower(), robot.armMotor.getCurrentPosition());
            telemetry.addData("Hand (A)", "position (%.2f), state (%s)", robot.handPosition, robot.getHandState());
            telemetry.addData("Gripper (X)", "position (%.2f), open (%b)", robot.gripperPosition, robot.gripperOpen);

            telemetry.addData("\nStatus", "Run Time: %.2f", runtime.seconds());
            telemetry.update();
        }
    }

    void updateDrive() {
        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper)
            crawlingMode = !crawlingMode;
        preciseMode = currentGamepad1.right_bumper;

        // Calculate drive and turn
        double driveInput = -gamepad1.left_stick_y;
        double turnInput = gamepad1.right_stick_x;
        double drive = driveInput;
        double turn = turnInput;

        if (crawlingMode) {
            drive = driveInput * MAX_CRAWL_SPEED;
            turn = turnInput * MAX_CRAWL_SPEED;
        }
        if (preciseMode) {
            drive = driveInput * MAX_PRECISE_SPEED;
            turn = turnInput * MAX_PRECISE_SPEED;
        }

        // Update motors power
        robot.driveRobot(drive, turn);
    }

    void updateArm() {
        // Update arm power
        if (currentGamepad2.left_trigger != 0) {
            if (robot.armMotor.getCurrentPosition() > 5)
                robot.setArmPower(-currentGamepad2.left_trigger * ARM_DOWN_MAX_POWER);
            else robot.setArmPower(0);
        } else {
            double armPower = -gamepad2.left_stick_y;
            if (armPower > 0) armPower *= ARM_MAX_POWER;
            else armPower *= ARM_DOWN_MAX_POWER;
            robot.setArmPowerOrHold(armPower, 300, !currentGamepad2.left_bumper);
        }

        // Reset arm encoder
        if (previousGamepad2.left_bumper && !currentGamepad2.left_bumper) {
            robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
            robot.setHandPosition(Range.clip(robot.handPosition + 0.05, 0, 1));
        } else if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
            robot.setHandPosition(Range.clip(robot.handPosition - 0.05, 0, 1));
        }
        // Change hand position using A, B, and Y buttons
        if (currentGamepad2.a && !previousGamepad2.a) {
            robot.setHandState(Robot.HandState.DOWN);
        } else if (currentGamepad2.b && !previousGamepad2.b) {
            robot.setHandState(Robot.HandState.BACKDROP);
        } else if (currentGamepad2.y && !previousGamepad2.y) {
            robot.setHandState(Robot.HandState.UP);
        }
        // Toggle gripper position using X button
        if (currentGamepad2.x && !previousGamepad2.x) {
            robot.setGripperState(!robot.gripperOpen);
        }
    }

    // TODO: determine helpfullness of this
    private double smooth(double x, double factor) {
        return Math.pow(Math.abs(x), factor) * Math.signum(x);
    }
}
