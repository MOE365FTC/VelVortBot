/* Copyright (c) 2015 Qualcomm Technologies Inc

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

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutonomousRed", group = "Auton")
//@Disabled                            // Comment this out to add to the opmode list
public class AutonRed extends LinearOpMode {
    DcMotor FR,FL,BR,BL,Shooter,Harvester;
    ModernRoboticsI2cColorSensor beaconSensor;
    ModernRoboticsI2cColorSensor lineSensor;
    I2cAddr beaconAddress = I2cAddr.create8bit(0x3c);
    I2cAddr lineAddress = I2cAddr.create8bit(0x4c);
    ModernRoboticsI2cGyro gyro;
    double shooterPower = 0.8;
    double harvesterPower = 0.6;
    double ticsPerInch = 38.5;
    double strafeTicsPerInch = 60;
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {

        // get a reference to our DeviceInterfaceModule object.
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        Shooter = hardwareMap.dcMotor.get("Sh");
        Harvester = hardwareMap.dcMotor.get("Ha");
        lineSensor = (ModernRoboticsI2cColorSensor)hardwareMap.colorSensor.get("lineSense");
        beaconSensor = (ModernRoboticsI2cColorSensor)hardwareMap.colorSensor.get("beaconSense");
        lineSensor.setI2cAddress(lineAddress);
        beaconSensor.setI2cAddress(beaconAddress);

        Shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        Harvester.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        Shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set the digital channel to output mode.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }
        telemetry.addData(">", "Gyro Calibrated, Tracking Started.");
        telemetry.update();
        waitForStart();
        /*****************************************************************/
        shootBalls();
        moveRightInches(5,0.5);
        FL.setPower(.25);
        BL.setPower(.25);
        while(opModeIsActive() && gyro.getIntegratedZValue() > -45){
            idle();
        }
        stopDrive();
        Thread.sleep(100);
        moveForwardInches(63,0.3);
        Thread.sleep(100);
        rightTurn(83-Math.abs(gyro.getIntegratedZValue()),0.2);
        Thread.sleep(100);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        moveLeftInches(4,0.5);
        Thread.sleep(100);
        startBackward(0.2);
        while(opModeIsActive() && lineSensor.green() == 0){
            idle();
        }
        stopDrive();
        Thread.sleep(100);
        pressBeacon(0.5);
        moveForwardInches(20,0.3);
        startForward(0.2);
        while(opModeIsActive() && lineSensor.green() == 0){
            idle();
        }
        stopDrive();
        moveBackwardInches(2,0.3);
        pressBeacon(0.5);
        moveBackwardInches(108,0.5);
         /*************************************************************/
    }

    public void pressBeacon(double speed){
        if(beaconSensor.red() > beaconSensor.blue()){//IF READING RED
            moveForwardInches(1,0.2);
            runtime.reset();
            startLeft(speed);
            while(opModeIsActive() && runtime.seconds() < 2){
                idle();
            }
            stopDrive();
            moveRightInches(6,0.5);
        }
        else{
            moveForwardInches(4,0.3);
            runtime.reset();
            startLeft(speed);
            while(opModeIsActive() && runtime.seconds() < 2){
                idle();
            }
            stopDrive();
            moveRightInches(6,0.5);
        }
    }

    public void shootBalls(){
        Shooter.setTargetPosition(1650);
        Shooter.setPower(shooterPower);
        while(opModeIsActive() && Shooter.isBusy()){
            idle();
        }
        Shooter.setPower(0);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < 2.0)){
            Harvester.setPower(harvesterPower);
            idle();
        }
        Harvester.setPower(0);
        Shooter.setTargetPosition(3300);
        Shooter.setPower(shooterPower);
        while(opModeIsActive() && Shooter.isBusy()){
            idle();
        }
        Shooter.setPower(0);
    }

    public void rightTurn(double angle, double speed){
        double target = gyro.getIntegratedZValue() - angle;
        FR.setPower(-speed);
        FL.setPower(speed);
        BR.setPower(-speed);
        BL.setPower(speed);
        while(opModeIsActive() && gyro.getIntegratedZValue() > target){
            idle();
        }
        stopDrive();
    }

    public void leftTurn(double angle, double speed){
        double target = gyro.getIntegratedZValue() + angle;
        FR.setPower(speed);
        FL.setPower(-speed);
        BR.setPower(speed);
        BL.setPower(-speed);
        while(opModeIsActive() && gyro.getIntegratedZValue() < target){
            idle();
        }
        stopDrive();
    }

    public void moveRightInches(double inches, double speed){
        double target = -inches * strafeTicsPerInch;
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(FR.getCurrentPosition()!=0){
            idle();
        }
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startRight(speed);
        while(opModeIsActive() && FR.getCurrentPosition() > target){
            idle();
        }
        stopDrive();
    }

    public void moveLeftInches(double inches, double speed){
        double target = inches * strafeTicsPerInch;
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(FR.getCurrentPosition()!=0){
            idle();
        }
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startLeft(speed);
        while(opModeIsActive() && FR.getCurrentPosition() < target){
            idle();
        }
        stopDrive();
    }

    public void moveForwardInches(double inches, double speed){
        double target = inches * ticsPerInch;
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startForward(speed);
        while(opModeIsActive() && FR.getCurrentPosition() < target){
            idle();
        }
        stopDrive();
    }

    public void moveBackwardInches(double inches, double speed){
        double target = inches*ticsPerInch;
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startBackward(speed);
        while(opModeIsActive() && FR.getCurrentPosition() > -target){
            idle();
        }
        stopDrive();
    }

    public void startForward(double speed){
        FR.setPower(speed);
        FL.setPower(speed);
        BR.setPower(speed);
        BL.setPower(speed);
    }

    public void startBackward(double speed){
        FR.setPower(-speed);
        FL.setPower(-speed);
        BR.setPower(-speed);
        BL.setPower(-speed);
    }

    public void startRight(double speed){
        FR.setPower(-speed);
        BL.setPower(-(speed+.1));
        BR.setPower(speed+.1);
        FL.setPower(speed);
    }

    public void startLeft(double speed){
        FR.setPower(speed+.1);
        BL.setPower(speed);
        FL.setPower(-(speed+.1));
        BR.setPower(-speed);
    }

    public void stopDrive(){
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }

}