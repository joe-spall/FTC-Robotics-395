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
import java.lang.Math;


/**
 * TeleOp Mode
 * <p>
 * Team 395 K9 Tele Op
 */
public class BirdmanMainWLift extends OpMode {


    DcMotorController wheelControllerFront;
    DcMotorController wheelControllerBack;
    DcMotorController liftController;
    DcMotor motorWheel0;
    DcMotor motorWheel1;
    DcMotor motorWheel2;
    DcMotor motorWheel3;
    DcMotor motorRotate;
    DcMotor motorLift;
    boolean rotateFast;
    
    boolean driveFast;




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
        wheelControllerFront = hardwareMap.dcMotorController.get("wheelControllerFront");
        wheelControllerBack = hardwareMap.dcMotorController.get("wheelControllerBack");

        //Rotate fast
        rotateFast = false;

      	//Lift init
        motorRotate = hardwareMap.dcMotor.get("motor_6");
        motorLift = hardwareMap.dcMotor.get("motor_7");

       
        driveFast = true;

        //Lift slide controller init
        liftController = hardwareMap.dcMotorController.get("liftController");


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
    
        boolean aPushFast = gamepad1.a;
        boolean bPushSlow = gamepad1.b;
       
        

        if(aPushFast)
        {
            driveFast = true;
        }
        
        if(bPushSlow)
        {
            driveFast = false;
        }
        
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
        if(driveFast)
        {
            // write the values to the motors
            motorWheel0.setPower(leftSide);
            motorWheel1.setPower(rightSide);
            motorWheel2.setPower(leftSide);
            motorWheel3.setPower(rightSide);
        }
        else
        {
            // write the values to the motors
            motorWheel0.setPower(leftSide/2);
            motorWheel1.setPower(rightSide/2);
            motorWheel2.setPower(leftSide/2);
            motorWheel3.setPower(rightSide/2);
        }
        
    
        /*
         * Send telemetry data back to driver station.
         */
        
        telemetry.addData("drive fast", "drive fast: " + Boolean.toString(driveFast));
        telemetry.addData("left side pwr",  "left side  pwr: " + String.format("%.2f", leftSide));
        telemetry.addData("right side pwr", "right side pwr: " + String.format("%.2f", rightSide));
        
        //Lift rotate
        float rotate = gamepad2.right_stick_y;
        boolean pushRotateSlowButton = gamepad2.a;
        boolean pushRotateFastButton = gamepad2.b;

        if(pushRotateSlowButton)
        {
            rotateFast = false;
        }

        if(pushRotateFastButton)
        {
            rotateFast = true;
        }

        telemetry.addData("rotate fast", "rotate fast: " + Boolean.toString(rotateFast));



        //Range and scale
        rotate = Range.clip(rotate, -1, 1);
        rotate = (float)scaleInput(rotate);


        //Set power
        if(rotateFast == true)
        {
            motorRotate.setPower(rotate);
            telemetry.addData("rotate pwr", "rotate pwr: " + String.format("%.2f", rotate));

        }
        else if(rotateFast == false)
        {
            motorRotate.setPower(rotate/2);
            telemetry.addData("rotate pwr", "rotate pwr: " + String.format("%.2f", rotate/2));
        }


        //Lift up
        float lift = (gamepad2.left_stick_y);

        //Range and scale
        lift = Range.clip(lift, -1, 1);
        lift = (float)scaleInput(lift);

        //Set power
        motorLift.setPower(lift);

		/*
		 * Send telemetry data back to driver station.
		 */
        

        telemetry.addData("lift pwr",  "lift pwr: " + String.format("%.2f", lift));

    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

        motorWheel0.setPower(0);
        motorWheel1.setPower(0);
        motorWheel2.setPower(0);
        motorWheel3.setPower(0);
        motorRotate.setPower(0);
        motorLift.setPower(0);

    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        
        double scaleOutput = Math.pow(dVal,3)

        return scaleOutput;
    }

}
