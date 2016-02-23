
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
public class BirdmanAutoBlueBlue extends OpMode {


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
    boolean checkEncoders;

    //In inches
    final static double WHEEL_DIAMETER = 4.8;
    final static double CIRCUMFRENCE = Math.PI*WHEEL_DIAMETER;
    final static double ENCODER_CPR = 1440;
    final static double LINEAR_DISTANCE_1 = 20;



    int state;


    /**
     * Constructor
     */
    public BirdmanAutoBlueBlue() {

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
        motorWheel0 = hardwareMap.dcMotor.get("motor_0");
        motorWheel1 = hardwareMap.dcMotor.get("motor_1");
        motorWheel2 = hardwareMap.dcMotor.get("motor_2");
        motorWheel3 = hardwareMap.dcMotor.get("motor_3");

        //Direction of wheels
        motorWheel0.setDirection(DcMotor.Direction.REVERSE);
        motorWheel2.setDirection(DcMotor.Direction.REVERSE);

        //Wheel controller init
        wheelControllerFront = hardwareMap.dcMotorController.get("wheelControllerFront");
        wheelControllerBack = hardwareMap.dcMotorController.get("wheelControllerBack");

        //Rotate fast
        rotateFast = false;

        checkEncoders = false;
        //Lift init
        motorRotate = hardwareMap.dcMotor.get("motor_4");
        motorLift = hardwareMap.dcMotor.get("motor_5");




        //Lift slide controller init
        liftController = hardwareMap.dcMotorController.get("liftController");

    }

    /*
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

        //Start
        if (state == 0) {
            changeEncoders("reset");
            state++;
        }//Drive forward
        else if (state == 1) {

            setMotorPosition(114,114);
            setMotorPower(0.3,0.3);
            changeEncoders("run");
            state++;
        }//Wait to finish
        else if (state == 2) {



        }//Stop
        else if (state == 3) {
            stop();
        }

        telemetry.addData("Text", "Auto");
        telemetry.addData("current state", state);


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
    }


    //Reset encoders
    public void changeEncoders(String change) {
        if(change.equals("reset"))
        {
            motorWheel0.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            motorWheel1.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            motorWheel2.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            motorWheel3.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        }

        if(change.equals("run"))
        {
            motorWheel0.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorWheel1.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorWheel2.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorWheel3.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        }
    }

    public void setMotorPower(double leftPow, double rightPow)
    {
        motorWheel0.setPower(leftPow);
        motorWheel1.setPower(rightPow);
        motorWheel2.setPower(leftPow);
        motorWheel3.setPower(rightPow);
    }

    public void setMotorPosition(double leftDis, double rightDis)
    {
        double leftStep = (leftDis/CIRCUMFRENCE)*ENCODER_CPR;
        double rightStep = (rightDis/CIRCUMFRENCE)*ENCODER_CPR;
        motorWheel0.setTargetPosition((int)leftStep);
        motorWheel1.setTargetPosition((int)rightStep);
        motorWheel2.setTargetPosition((int)leftStep);
        motorWheel3.setTargetPosition((int)rightStep);

    }









}