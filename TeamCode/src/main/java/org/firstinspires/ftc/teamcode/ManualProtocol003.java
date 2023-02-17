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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp
public class ManualProtocol003 extends Main003 {
    private double left_front_power = 0;
    private double right_front_power = 0;
    private double left_back_power = 0;
    private double right_back_power = 0;

    private double linear_actuator_power = 0;
    private double gripper_arm_power = 0;

    @Override
    public void runOpMode() {
        initMaths();
        initHardware();
        initManualModes();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                setMotorForces();
                setGripperForces();

                telemetry.update();

                idle();
            }
        }
    }

    public void setMotorForces() {
        if(!noNullHardware()) return;

        boolean gamepadTwoY = gamepad2.y;
        boolean gamepadTwoX = gamepad2.x;

        double gamepadOneLeftX = gamepad1.left_stick_x;
        double gamepadOneLeftY = gamepad1.left_stick_y;

        double gamepadTwoLeftY = gamepad2.left_stick_y;
        double gamepadTwoRightY = gamepad2.right_stick_y;

        double rAxis = gamepad1.right_trigger - gamepad1.left_trigger;

        int quad = math.getQuad(gamepadOneLeftX, gamepadOneLeftY);
        double theta = math.theta(gamepadOneLeftX, gamepadOneLeftY, quad);
        double z = (double) Math.sqrt(Math.pow(gamepadOneLeftX, 2) + Math.pow(gamepadOneLeftY, 2)) * math.wheelControlMultiplier;

        left_front_power = -math.getWheelForceManual(gamepadOneLeftX, gamepadOneLeftY, 1, rAxis, theta, z);
        right_front_power = math.getWheelForceManual(gamepadOneLeftX, gamepadOneLeftY, 2, rAxis, theta, z);
        left_back_power = -math.getWheelForceManual(gamepadOneLeftX, gamepadOneLeftY, 3, rAxis, theta, z);
        right_back_power = math.getWheelForceManual(gamepadOneLeftX, gamepadOneLeftY, 4, rAxis, theta, z);

        linear_actuator_power = -math.getLinearActuatorForce(gamepadTwoLeftY);
        gripper_arm_power = math.getGripperArmForce(gamepadTwoRightY);

        telemetry.addData("lpb", left_back_power + " q: " + quad + " theta: " + theta + "x: " + gamepadOneLeftX + "y: " + gamepadOneLeftY + "x2: " + gamepad1.right_stick_x);
        telemetry.addData("lpf", left_front_power);
        telemetry.addData("rpb", right_back_power);
        telemetry.addData("rpf", right_front_power);

        telemetry.addData("TICKS TO KNOW", gripper_arm.getCurrentPosition());

        telemetry.addData("linear actuator power", linear_actuator_power);
        telemetry.addData("gripper arm power", gripper_arm_power);

        left_front.setVelocity(left_front_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);
        right_front.setVelocity(right_front_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);
        left_back.setVelocity(left_back_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);
        right_back.setVelocity(right_back_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);

        linear_actuator.setVelocity(linear_actuator_power * MAX_NUM_TICKS_ACTUATOR * ACTUATOR_RPM);

        gripper_arm.setVelocity(gripper_arm_power * MAX_NUM_TICKS_ARM * ARM_RPM);

        if (gripper_arm.getCurrentPosition() <= -1070) {
            gripper_rotator.setTargetPosition(1948);
            gripper_rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            gripper_rotator.setVelocity(5844);
        }


        if (gripper_arm.getCurrentPosition() >= -1070) {
            gripper_rotator.setTargetPosition(0);
            gripper_rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            gripper_rotator.setVelocity(5844);
        }
    }

    public void setGripperForces() {
        boolean gripperClose = gamepad2.a;
        boolean gripperOpen = gamepad2.b;
        if (gripperClose) {
            gripper.setPosition(1);
        }
        if (gripperOpen) {
            gripper.setPosition(0.65);
        }

        double triggers = gamepad2.right_trigger - gamepad2.left_trigger;
        if ((triggers > 0) || (triggers < 0)) {
            gripper_wrist.setPosition(triggers);
        } else if (triggers == 0) {
            gripper_wrist.setPosition(0.25);
        }
    }
}