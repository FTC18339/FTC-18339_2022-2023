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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


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

public class Main002 extends LinearOpMode {
    public DcMotorEx left_front;
    public DcMotorEx right_front;
    public DcMotorEx left_back;
    public DcMotorEx right_back;

    public DcMotorEx linear_actuator;
    public DcMotorEx gripper_arm;

    public Servo gripper;
    public Servo gripper_wrist;

    public final float MAX_NUM_TICKS_MOVEMENT = 537.7f;
    public final float MAX_NUM_TICKS_ACTUATOR = 384.5f;
    public final float MAX_NUM_TICKS_ARM = 5281.1f;

    public final float MOVEMENT_RPM = 25;
    public final float ACTUATOR_RPM = 180;
    public final float ARM_RPM = 10;


    public Algorithms002 math;

    @Override
    public void runOpMode() { }

    public void initMaths() {
        math = new Algorithms002();
        math.Initialize();
    }

    public void initHardware() {
        left_front = hardwareMap.get(DcMotorEx.class, "left_front");
        right_front = hardwareMap.get(DcMotorEx.class, "right_front");
        left_back = hardwareMap.get(DcMotorEx.class, "left_back");
        right_back = hardwareMap.get(DcMotorEx.class, "right_back");

        linear_actuator = hardwareMap.get(DcMotorEx.class, "linear_actuator");
        gripper_arm = hardwareMap.get(DcMotorEx.class, "gripper_arm");

        gripper = hardwareMap.get(Servo.class, "gripper");
        gripper_wrist = hardwareMap.get(Servo.class, "gripper_wrist");

        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linear_actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gripper_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // gripper.setDirection();
    }

    public void initManualModes() {
        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linear_actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gripper_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stopMotors() {
        left_front.setVelocity(0);
        right_front.setVelocity(0);
        left_back.setVelocity(0);
        right_back.setVelocity(0);

        linear_actuator.setVelocity(0);
        gripper_arm.setVelocity(0);
    }

    public boolean noNullHardware() {
        return (left_back != null && left_front != null && right_back != null && right_front != null && linear_actuator != null && gripper_arm != null && gripper != null && gripper_wrist != null);
    }
}