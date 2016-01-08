
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
public class BirdmanAutoRedBlue extends OpMode {
    
    
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
    
    /**
     * Constructor
     */
    public BirdmanAutoRedBlue() {
        
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

        //Read and write
      //  motorWheel0.setMode(DcMotorController.RunMode.READ_WRITE);

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
        motorWheel0.setPower(0);
        motorWheel1.setPower(0);
        motorWheel2.setPower(0);
        motorWheel3.setPower(0);
    }

    //Reset encoders
    public void resetEncoders()
    {
        motorWheel0.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorWheel1.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorWheel2.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorWheel3.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    //Start encoders
    public void startEncoders()
    {
        motorWheel0.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorWheel1.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorWheel2.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorWheel3.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }


    
    //Motor power set
    public void motorPower(double powerLeft, double powerRight)
    {
        motorWheel0.setPower(powerLeft);
        motorWheel1.setPower(powerRight);
    }
    
    //Motor encoder check
    public boolean motorEncoderCheck(double goalLeft, double goalRight)
    {   //Check success
        boolean returnState = false;
        
        //Check position
        if(motorWheel1.getCurrentPosition() >= goalRight && motorWheel0.getCurrentPosition() <= goalLeft)
        {
            returnState = true;
        }
        
        //Return success
        return returnState;
    }
}
