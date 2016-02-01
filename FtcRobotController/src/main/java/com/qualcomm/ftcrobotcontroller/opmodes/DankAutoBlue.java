package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;


/**
 * Created by Bondi on 1/9/2016.
 */
public class DankAutoBlue extends OpMode {

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



        DcMotorController.DeviceMode devMode;
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

        int state;
        int numLoops;

        double currentLeft=0.0;
        double currentRight= 0.0;
        double maxPower = 0.4;

        int dist1 = 2200;
        int dist2 = 1290;
        int dist3 = 5000;
        int armDist = 1000;

        /**
         * Constructor
         */
        public DankAutoBlue() {

        }

        /*
         *
         * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
         */
        @Override
        public void init() {
            //State control
            state = 0;
            devMode = DcMotorController.DeviceMode.READ_WRITE;
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

            //Start
            if(state == 0)
            {
                long startTime=0;
                state++;
            }//Drive forward
            else if(state == 1)
            {
                long startTime=System.currentTimeMillis();
                long endTime1=startTime+dist1;






                    while(System.currentTimeMillis()<endTime1-500)
                    {
                        if(currentLeft<maxPower)currentLeft+=0.01;
                        if(currentRight<maxPower)currentRight+=0.01;
                        motorPower(currentLeft,currentRight);

                    }

                while(System.currentTimeMillis()<endTime1)
                {
                    if(currentLeft>0)currentLeft-=0.01;
                    if(currentRight>0)currentRight-=0.01;
                    motorPower(currentLeft,currentRight);

                }
                state++;


            }//Turn
            else if(state == 2)
            {
                long startTime=System.currentTimeMillis();

                long endTime2=startTime+dist2;




                while(System.currentTimeMillis()<endTime2-500)
                {
                    if(currentLeft>-maxPower)currentLeft-=0.01;
                    if(currentRight<maxPower)currentRight+=0.01;
                    motorPower(currentLeft,currentRight);

                }

                while(System.currentTimeMillis()<endTime2)
                {
                    if(currentLeft<0)currentLeft+=0.01;
                    if(currentRight>0)currentRight-=0.01;
                    motorPower(currentLeft,currentRight);

                }
                while(System.currentTimeMillis()<endTime2+armDist)
                {
                    motorRotate.setPower(1);
                }

                state++;
            }//Drive Forwards again
            else if(state == 3)
            {
                long startTime=System.currentTimeMillis();

                long endTime3=startTime+dist3;


                while(System.currentTimeMillis()<endTime3-500)
                {
                    if(currentLeft<maxPower)currentLeft+=0.01;
                    if(currentRight<maxPower)currentRight+=0.01;
                    motorPower(currentLeft,currentRight);

                }

                while(System.currentTimeMillis()<endTime3) {
                    if (currentLeft > 0) currentLeft -= 0.01;
                    if (currentRight > 0) currentRight -= 0.01;
                    motorPower(currentLeft, currentRight);

                }
                state++;
            }//stop
            else if(state == 4)
            {
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



        //Motor power set
        public void motorPower(double powerLeft, double powerRight)
        {

                motorWheel0.setPower(powerLeft);
                motorWheel1.setPower(powerRight);
                motorWheel2.setPower(powerLeft);
                motorWheel3.setPower(powerRight);


        }


    }

