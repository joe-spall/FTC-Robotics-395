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
public class BirdmanTest extends OpMode {


    DcMotorController wheelController;
    DcMotor motorWheel0;





    /**
     * Constructor
     */
    public BirdmanTest() {

    }

    /*
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
     */
    @Override
    public void init() {

        //Wheels init
        motorWheel0 = hardwareMap.dcMotor.get("motor_1");

        //Direction of wheels
        motorWheel0.setDirection(DcMotor.Direction.REVERSE);

        //Wheel controller init
        wheelController = hardwareMap.dcMotorController.get("wheelController");


        //Wheel mode
        motorWheel0.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);


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
		 *
		 *                                       Front
		 *
		 *
		 *              Left Joystick                              Right Joystick
		 *                                         |
		 *                 Lift up                 |              Lift rotate up
		 *                   ^ ^                   |                    ^ ^
		 *                  -----                  |                   -----
		 *                |       |                |                 |       |
		 *             < |         | >             |              < |         | >
		 *             < |         | >             |              < |         | >
		 *                |       |                |                 |       |
		 *                  -----                  |                   -----
		 *                   v v                   |                    v v
		 *                Lift down                |              Lift rotate down
		 *
		 *
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
        float throttle = gamepad1.left_stick_y;



        // clip the right/left values so that the values never exceed +/- 1
        throttle = Range.clip(throttle, -1, 1);


        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        throttle = (float)scaleInput(throttle);


        // write the values to the motors
        motorWheel0.setPower(throttle);





		/*
		 * Send telemetry data back to driver station.
		 */
        telemetry.addData("left wheel pwr",  "left side  pwr: " + String.format("%.2f", throttle));


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
