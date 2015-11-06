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



/**
 * TeleOp Mode
 * <p>
 * Team 395 K9 Tele Op
 */
public class K9AutoRedFloorBirdman extends OpMode {


    DcMotorController wheelController;
    DcMotorController armController;
    DcMotor motorArmTop;
    DcMotor motorArmBottom;
    DcMotor motorRight;
    DcMotor motorLeft;
    int state;

    /**
     * Constructor
     */
    public K9AutoRedFloorBirdman() {

    }

    /*
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
     */
    @Override
    public void init() {
        //State control
        state = 0;

        //Wheels init
        motorRight = hardwareMap.dcMotor.get("motor_2");
        motorLeft = hardwareMap.dcMotor.get("motor_1");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        wheelController = hardwareMap.dcMotorController.get("wheelsController");
        motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);

        //Arm init
        motorArmTop = hardwareMap.dcMotor.get("motor_3");
        motorArmBottom = hardwareMap.dcMotor.get("motor_4");
        armController = hardwareMap.dcMotorController.get("armController");
        motorArmTop.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorArmBottom.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

    }

    /*
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

        //Start
        if(state == 0)
        {
            resetEncoders();
            state++;
        }//Drive forward
        else if(state == 1)
        {
            startEncoders();
            motorPower(1.0,-1.0);
            if(motorEncoderCheck(2000,-2000))
            {
                resetEncoders();
                motorPower(0.0,0.0);
                state++;
            }
        }//Turn
        else if(state == 2)
        {

            motorPower(1.0,1.0);
            if(motorEncoderCheck(100,100))
            {
                resetEncoders();
                motorPower(0.0, 0.0);
                state++;
            }
        }//Stop
        else if(state == 3)
        {
            stop();
        }




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
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

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

    //Reset encoders
    public void resetEncoders()
    {
        motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    //Start encoders
    public void startEncoders()
    {
        motorRight.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorLeft.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    //Motor power set
    public void motorPower(double powerLeft, double powerRight)
    {
        motorLeft.setPower(powerLeft);
        motorRight.setPower(powerRight);
    }

    //Motor encoder check
    public boolean motorEncoderCheck(double goalLeft, double goalRight)
    {   //Check success
        boolean returnState = false;

        //Check position
        if(motorRight.getCurrentPosition() >= goalRight && motorLeft.getCurrentPosition() <= goalLeft)
        {
            returnState = true;
        }

        //Return success
        return returnState;
    }
}
