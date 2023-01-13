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
public class ManualProtocol002 extends Main002 {
    private double left_front_power = 0;
    private double right_front_power = 0;
    private double left_back_power = 0;
    private double right_back_power = 0;

    private double linear_actuator_power = 0;
    private double gripper_arm_power = 0;

    boolean gripOn = false;

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

        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;

        double yOne = gamepad2.left_stick_y;
        double yTwo = gamepad2.right_stick_y;

        double rAxis = gamepad1.right_trigger - gamepad1.left_trigger;

        int quad = math.getQuad(x, y);
        double theta = math.theta(x, y, quad);
        double z = (double) Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)) * math.controlMultiplier;

        left_front_power = -math.getWheelForceManual(x, y, 1, rAxis, theta, z);
        right_front_power = math.getWheelForceManual(x, y, 2, rAxis, theta, z);
        left_back_power = -math.getWheelForceManual(x, y, 3, rAxis, theta, z);
        right_back_power = math.getWheelForceManual(x, y, 4, rAxis, theta, z);

        linear_actuator_power = math.getLinearActuatorForce(yOne);
        gripper_arm_power = math.getGripperArmForce(yTwo);

        // these two are  here to make the motors move by the magnitude of the stick (test the above first, then possible move to this)
        // linear_actuator_power = yOne;
        // gripper_arm_power = yTwo;

        telemetry.addData("lpb", left_back_power + " q: " + quad + " theta: " + theta + "x: " + x + "y: " + y + "x2: " + gamepad1.right_stick_x);
        telemetry.addData("lpf", left_front_power);
        telemetry.addData("rpb", right_back_power);
        telemetry.addData("rpf", right_front_power);

        telemetry.addData("lap", linear_actuator_power);
        telemetry.addData("gmp", gripper_arm_power);

        left_front.setVelocity(left_front_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);
        right_front.setVelocity(right_front_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);
        left_back.setVelocity(left_back_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);
        right_back.setVelocity(right_back_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);

        linear_actuator.setVelocity(linear_actuator_power * MAX_NUM_TICKS_ACTUATOR * ACTUATOR_RPM);
        gripper_arm.setVelocity(gripper_arm_power * MAX_NUM_TICKS_ARM * ARM_RPM);
    }

    public void setGripperForces() {
        double triggerGrip = gamepad2.right_trigger;
        if (triggerGrip != 0) {
            if (!gripOn) {
                gripper.setPosition(1);
            } else if (gripOn) {
                gripper.setPosition(0);
            }
        }
    }
}