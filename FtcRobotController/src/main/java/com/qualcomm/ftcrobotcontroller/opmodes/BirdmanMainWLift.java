/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


/**
 * TeleOp Mode
 * <p>
 * Team 395 K9 Tele Op
 */
public class BirdmanMainWLift extends OpMode {


    DcMotorController wheelControllerLeft;
    DcMotorController wheelControllerRight;
    DcMotorController liftSlideController;
    DcMotorController liftRotateController;
    DcMotor motorWheel0;
    DcMotor motorWheel1;
    DcMotor motorWheel2;
    DcMotor motorWheel3;
    DcMotor motorLiftUp;
    DcMotor motorLiftDown;
    DcMotor motorLiftRotate;




    /**
     * Constructor
     */
    public BirdmanMainWLift() {

    }

    /*
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
     */
    @Override
    public void init() {

        //Wheels init
        motorWheel0 = hardwareMap.dcMotor.get("motor_1");
        motorWheel1 = hardwareMap.dcMotor.get("motor_2");
        motorWheel2 = hardwareMap.dcMotor.get("motor_3");
        motorWheel3 = hardwareMap.dcMotor.get("motor_4");

        //Direction of wheels
        motorWheel0.setDirection(DcMotor.Direction.REVERSE);
        motorWheel2.setDirection(DcMotor.Direction.REVERSE);

        //Wheel controller init
        wheelControllerLeft = hardwareMap.dcMotorController.get("wheelControllerLeft");
        wheelControllerRight = hardwareMap.dcMotorController.get("wheelControllerRight");

        //Wheel mode
        motorWheel0.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorWheel1.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorWheel2.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorWheel3.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

      	//Lift init
        motorLiftUp = hardwareMap.dcMotor.get("motor_5");
        motorLiftDown = hardwareMap.dcMotor.get("motor_6");
        motorLiftRotate = hardwareMap.dcMotor.get("motor_6");

        //Reverse motor
        motorLiftDown.setDirection(DcMotor.Direction.REVERSE);

        //Lift slide controller init
        liftSlideController = hardwareMap.dcMotorController.get("liftSlideController");
        liftRotateController = hardwareMap.dcMotorController.get("liftRotateController");

        //Lift mode
        motorLiftUp.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorLiftDown.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorLiftRotate.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);


    }

    /*
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

		/*
		 * Layout
		 *
		 *
		 *
		 *           0--------1
		 *           |        |
		 *           |        |
		 *           |        |
		 *           |        |
		 *           |        |
		 *           2--------3
	     *
		 *
         *
	     *
		 *
		 * Gamepad 1
		 *
		 *
		 *                                       Front
		 *
		 *
		 *              Left Joystick                              Right Joystick
		 *                                         |
		 *              Moves Forward              |
		 *                   ^ ^                   |                    ^ ^
		 *                  -----                  |                   -----
		 *                |       |                |                 |       |
		 *             < |         | >             | Rotates Left < |         | > Rotates Right
		 *             < |         | >             |              < |         | >
		 *                |       |                |                 |       |
		 *                  -----                  |                   -----
		 *                   v v                   |                    v v
		 *                Moves Back               |
		 *
		 *
		 *
		 *
		 *
		 * Gamepad 2
		 *
		 *
		 *                                        Back
		 *
		 *
		 *
		 *               Right Trigger              |                 Left Trigger
		 *                   ____                   |                    ____
		 *                  |    |                  |                   |    |
		 *                 |      |                 |                  |      |
		 *                |        |                |                 |        |
		 *                ----------                |                 ----------
		 *                   Lift                   |                   Lower
		 *                                          |
		 *                                          |
		 *                                          |
		 *                Right Bumper              |                 Left Bumper
		 *                 _________                |                  _________
		 *                |         |               |                 |         |
		 *                |         |               |                 |         |
		 *                -----------               |                 -----------
		 *                 Rotate up                |                 Rotate down
		 *
		 *
		 *
		 *
		 *
		 */

        // throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
        // 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left
        // and 1 is full right
        float throttle = -gamepad1.left_stick_y;
        float direction = -gamepad1.right_stick_x;
        float rightSide = throttle - direction;
        float leftSide = throttle + direction;


        // clip the right/left values so that the values never exceed +/- 1
        rightSide = Range.clip(rightSide, -1, 1);
        leftSide = Range.clip(leftSide, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        rightSide = (float)scaleInput(rightSide);
        leftSide =  (float)scaleInput(leftSide);

        // write the values to the motors
        motorWheel0.setPower(leftSide);
        motorWheel1.setPower(rightSide);
        motorWheel2.setPower(leftSide);
        motorWheel3.setPower(rightSide);


      	//Lift raising
        float liftUp = gamepad2.right_trigger;

        //Range and scale
        liftUp = Range.clip(liftUp, -1, 1);
        liftUp = (float)scaleInput(liftUp);

        //Set power
        motorLiftUp.setPower(liftUp);
        motorLiftDown.setPower(-liftUp);

        //Lift lower
        float liftDown = gamepad2.left_trigger;

        //Range and scale
        liftDown = Range.clip(liftDown, -1, 1);
        liftDown = (float)scaleInput(liftDown);

        //Set power
        motorLiftUp.setPower(-liftDown);
        motorLiftDown.setPower(liftDown);

        //Lift rotate up
        float liftRotateUp = gamepad2.right_stick_y;

        //Range and scale
        liftUp = Range.clip(liftUp, -1, 1);
        liftUp = (float)scaleInput(liftUp);

        //Set power
        motorLiftUp.setPower(liftUp);
        motorLiftDown.setPower(-liftUp);

        //Lift raising
        float liftDown = gamepad2.left_trigger;

        //Range and scale
        liftDown = Range.clip(liftDown, -1, 1);
        liftDown = (float)scaleInput(liftDown);

        //Set power
        motorLiftUp.setPower(-liftDown);
        motorLiftDown.setPower(liftDown);



		/*
		 * Send telemetry data back to driver station.
		 */
        telemetry.addData("left wheel pwr",  "left side  pwr: " + String.format("%.2f", leftSide));
        telemetry.addData("right wheel pwr", "right side pwr: " + String.format("%.2f", rightSide));
        telemetry.addData("top arm pwr",  "top arm pwr: " + String.format("%.2f", armTopPower));
        telemetry.addData("bottom arm pwr",  "bottom arm pwr: " + String.format("%.2f", armBottomPower));

    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }

}
