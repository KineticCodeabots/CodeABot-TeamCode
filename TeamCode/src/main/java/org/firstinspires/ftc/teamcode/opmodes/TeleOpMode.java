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
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOpMode")
public class TeleOpMode extends LinearOpMode {
    // Constants
    final private double DRIVE_SMOOTHING = 2;
    final private double TURN_SMOOTHING = 2;
    final private double ARM_MAX_POWER = 0.3;
    final private double HAND_GAIN = 0.01;

    // Gamepads to determine state changes.
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    private ElapsedTime runtime = new ElapsedTime();
    private Robot robot = new Robot(this);

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
            // Calculate drive and turn
            double drive = smooth(-gamepad1.left_stick_y, DRIVE_SMOOTHING);
            double turn = smooth(gamepad1.right_stick_x, TURN_SMOOTHING);
            // Update motors power
            robot.driveRobot(drive, turn);
            // Update arm power
            robot.setArmPower(gamepad2.left_stick_y * ARM_MAX_POWER);
            // Toggle hand position from down to background using A button
            if (currentGamepad2.a && !previousGamepad2.a) {
                Robot.HandState handState = robot.getHandState();
                if (handState == Robot.HandState.BACKBOARD) {
                    robot.setHandState(Robot.HandState.DOWN);
                } else {
                    robot.setHandState(Robot.HandState.BACKBOARD);
                }
            }
            // Toggle gripper position using X button
            if (currentGamepad2.x && !previousGamepad2.x) {
                robot.setGripperState(!robot.gripperOpen);
            }
            // Send telemetry
            robot.sendTelemetry();
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

    // TODO: determine helpfullness of this
    private double smooth(double x, double factor) {
        return Math.pow(Math.abs(x), factor) * Math.signum(x);
    }
}
