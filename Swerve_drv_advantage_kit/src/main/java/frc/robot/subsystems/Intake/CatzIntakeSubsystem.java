// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.CatzConstants;
import frc.robot.Utils.CatzStateUtil;



public class CatzIntakeSubsystem extends SubsystemBase {
    
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public static CatzIntakeSubsystem instance;

    //----------------------------------------------------------------------------------------------
    //
    //  Roller Control Constants
    //
    //----------------------------------------------------------------------------------------------

    private final double ROLLERS_PWR_CUBE_IN = -0.8;   
    private final double ROLLERS_PWR_CONE_IN =  1.0; //TBD decide pwrs for all cube cone scoring rollers

    private final double ROLLERS_PWR_CUBE_OUT =  1.0;   
    private final double ROLLERS_PWR_CONE_OUT = -0.5;

    

    //----------------------------------------------------------------------------------------------
    //  Wrist encoder & Position Values
    //----------------------------------------------------------------------------------------------
    private final int    WRIST_ENC_CAN_ID = 13; 


    private final double ENC_TO_INTAKE_GEAR_RATIO =  46.0/18.0;
    private final double WRIST_CNTS_PER_DEGREE    = 46.459; //(4096.0 * ENC_TO_INTAKE_GEAR_RATIO) / 360.0;


    private final double MANUAL_HOLD_STEP_SIZE = 1.5;       

    //TBD - ADD comment for ref point
    //Reference Point = wrist would be slight above "Parallel to the ground"
    public final double CENTER_OF_MASS_OFFSET_DEG     = 177.0; 
    public final double WRIST_ABS_ENC_OFFSET_DEG = 0.0; //Set to make stow pos equal to 0
    public final double WRIST_ABS_ENC_OFFSET = WRIST_ABS_ENC_OFFSET_DEG * WRIST_CNTS_PER_DEGREE;//-989.0; //Negative value means abs enc 0 is above intake angle 0   
    
    public final double STOW_ENC_POS               =  0.0 + WRIST_ABS_ENC_OFFSET_DEG;//4872.0 + WRIST_ABS_ENC_OFFSET; //3883
    private final double STOW_CUTOFF                =  -7.232 + WRIST_ABS_ENC_OFFSET_DEG;// + WRIST_ABS_ENC_OFFSET; //3670

    public final double INTAKE_CUBE_ENC_POS        =  -147.000 + WRIST_ABS_ENC_OFFSET_DEG;//1324.0 + WRIST_ABS_ENC_OFFSET;    //-335
    public final double INTAKE_PICKUP_CONE_ENC_POS_GROUND =  -184.524 + WRIST_ABS_ENC_OFFSET_DEG;//-306.0  + WRIST_ABS_ENC_OFFSET;  //-1295  
    public final double INTAKE_PICKUP_CONE_ENC_POS_SINGLE =  -116.400 + WRIST_ABS_ENC_OFFSET_DEG;//2089.0 + WRIST_ABS_ENC_OFFSET;  //1100

    public final double SCORE_CUBE_ENC_POS         =  -104.000 + WRIST_ABS_ENC_OFFSET_DEG;//1859.0 + WRIST_ABS_ENC_OFFSET;  //870     // Applies to low-mid-high

    public final double SCORE_CONE_HIGH_ENC_POS    =  -153.000 + WRIST_ABS_ENC_OFFSET_DEG;//289.0 + WRIST_ABS_ENC_OFFSET;  //-700
    public final double SCORE_CONE_MID_ENC_POS     = INTAKE_PICKUP_CONE_ENC_POS_GROUND; //TBD verify if its the same as high
    public final double SCORE_CONE_LOW_ENC_POS     = INTAKE_PICKUP_CONE_ENC_POS_GROUND; //TBD


    private final double SOFT_LIMIT_FORWARD = 0.0; //4876  + WRIST_ABS_ENC_OFFSET;  //3887
    private final double SOFT_LIMIT_REVERSE = -8900.0; //-798.0 + WRIST_ABS_ENC_OFFSET; //-1787     //TBD

    private final double GROSS_kP = 0.002472;//0.00009; 
    private final double GROSS_kI = 0.0;//000040;
    private final double GROSS_kD = 0.000291;//0.000007;

    private final double FINE_kP = 0.005234;//0.00009; 
    private final double FINE_kI = 0.0;//000008;
    private final double FINE_kD = 0.000291;//0.000007;
    
    private final double MAX_GRAVITY_FF = 0.055; //0.09

    
    private PIDController intakePID;
    
    public Boolean  pidEnable = false;

    private double   targetPositionDeg = STOW_ENC_POS;

    private double   targetPower = 0.0;
    private double   prevTargetPwr = 0.0;

    private double currentPosition = -999.0;
    private double positionError = -999.0; 
    public double prevCurrentPosition = -999.0;

    private boolean intakeInPosition = false;

    private final double INTAKE_POS_ERROR_THRESHOLD_DEG = 5.0;
    private final double PID_FINE_GROSS_THRESHOLD_DEG = 17.0;

    private double pidPower = 0.0;
    private double ffPower = 0.0;

    private int numConsectSamples = 0;




  private CatzIntakeSubsystem() 
  {
    switch(CatzConstants.currentMode)
    {
        case REAL:
            io = new IntakeIOReal();
            break;
        case SIM :
            io = new IntakeIOSim();
            break;
        default:
            io = new IntakeIOReal() {};
            break;
    }


        intakePID = new PIDController(GROSS_kP, GROSS_kI, GROSS_kD);
  }

  @Override
  public void periodic() 
  {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Intake", inputs);

  }

    /*----------------------------------------------------------------------------------------------
    *
    *  Utilities - PID 
    *
    *---------------------------------------------------------------------------------------------*/
    public void resetPID(){
      pidEnable = false;
      intakePID.reset();
    }

    public void enablePID(boolean set){
        pidEnable = set;
    }

    public boolean getPIDEnabled(){
        return pidEnable;
    }
    public void intakePIDLoopFunction()
    {
        //----------------------------------------------------------------------------------
        //  Chk if at final position
        //----------------------------------------------------------------------------------
        currentPosition = inputs.wristPosEnc / WRIST_CNTS_PER_DEGREE;
        positionError = currentPosition - targetPositionDeg;


        if  ((Math.abs(positionError) <= INTAKE_POS_ERROR_THRESHOLD_DEG))
        {
            numConsectSamples++;
            if(numConsectSamples >= 1)
            {   
                intakeInPosition = true;
            }
        }
        else
        {
            numConsectSamples = 0;
        }
        
        
        if(Math.abs(positionError) >= PID_FINE_GROSS_THRESHOLD_DEG)
        {
            intakePID.setP(GROSS_kP);
            intakePID.setI(GROSS_kI);
            intakePID.setD(GROSS_kD);
        }
        else if(Math.abs(positionError) < PID_FINE_GROSS_THRESHOLD_DEG)
        {
            intakePID.setP(FINE_kP);
            intakePID.setI(FINE_kI);
            intakePID.setD(FINE_kD);
        }

        pidPower = intakePID.calculate(currentPosition, targetPositionDeg);
        ffPower = calculateGravityFF();
        targetPower = pidPower + ffPower;

        //-------------------------------------------------------------
        //  checking if we did not get updated position value(Sampling Issue).
        //  If no change in position, this give invalid target power(kD issue).
        //  Therefore, go with prev targetPower Value.
        //-------------------------------------------------------------------
        if(prevCurrentPosition == currentPosition)
        {
            targetPower = prevTargetPwr;
        }

        //----------------------------------------------------------------------------------
        //  If we are going to Stow Position & have passed the power cutoff angle, set
        //  power to 0, otherwise calculate new motor power based on position error and 
        //  current angle
        //----------------------------------------------------------------------------------
        if(targetPositionDeg == STOW_ENC_POS && currentPosition > STOW_CUTOFF)
        {
            targetPower = 0.0;
        }
        wristSetPercentOuput(targetPower);

        prevCurrentPosition = currentPosition;
        prevTargetPwr = targetPower;
    }

    public void manualHoldingFunction(double wristPwr)
    {
        if(wristPwr > 0)
        {
          targetPositionDeg = Math.min((targetPositionDeg + wristPwr * MANUAL_HOLD_STEP_SIZE), SOFT_LIMIT_FORWARD);
        }
        else
        {
          targetPositionDeg = Math.max((targetPositionDeg + wristPwr * MANUAL_HOLD_STEP_SIZE), SOFT_LIMIT_REVERSE);
        }
        prevCurrentPosition = -prevCurrentPosition; //intialize for first time through thread loop, that checks stale position values
    }

    /*----------------------------------------------------------------------------------------------
    *
    *  Utilities - Rollers
    *
    *---------------------------------------------------------------------------------------------*/
    public void intakeRollerFunctionIN()
    {
        if(CatzStateUtil.currentGamePieceState == CatzStateUtil.GamePieceState.CUBE) {
        rollersInCube();
        }
        else {
        rollersInCone();
        }
    }
    public void intakeRollerFunctionOUT()
    {
        if(CatzStateUtil.currentGamePieceState == CatzStateUtil.GamePieceState.CUBE) {
        rollersOutCube();
        }
        else {
        rollersOutCone();
        }
    }
    
    public void intakeRollersOff()
    {
        io.rollersOffIO();
    }

    private void rollersInCube()
    {
        io.rollersOnIO(ROLLERS_PWR_CUBE_IN);
    }

    private void rollersOutCube()
    {
        io.rollersOnIO(ROLLERS_PWR_CUBE_OUT);
    }

    private void rollersInCone()
    {
        io.rollersOnIO(ROLLERS_PWR_CONE_IN);
    }

    private void rollersOutCone()
    {
        io.rollersOnIO(ROLLERS_PWR_CONE_OUT);
    }
    

    /*----------------------------------------------------------------------------------------------
    *
    *  Utilities - Wrist
    *
    *---------------------------------------------------------------------------------------------*/
    public void intakeManualHolding(double targetHoldingPwr)
    {
        io.intakeManualHoldingIO(targetHoldingPwr);
    }

    public void wristSetPercentOuput(double targetPwr)
    {
        io.wristSetPercentOuputIO(targetPwr);
    }

    public double calcWristAngle()
    {
        double wristAngle = ((inputs.wristPosEnc / WRIST_CNTS_PER_DEGREE) - WRIST_ABS_ENC_OFFSET_DEG);
        return wristAngle;
    }

    public double getWristPosition(){
        return inputs.wristPosEnc;
    }
    
    public double calculateGravityFF()
    {
        double radians = Math.toRadians(calcWristAngle() - CENTER_OF_MASS_OFFSET_DEG);
        double cosineScalar = Math.cos(radians);
        
        return MAX_GRAVITY_FF * cosineScalar;
    }

    public void setTargetPositionDegState(double stateSetTargetPositionDeg)
    {
        targetPositionDeg = stateSetTargetPositionDeg;
    }

    public double intakeWristTemp()
    {
        return inputs.wristTemp;
    }
    
    public void shuffleboardIntake()
    {
    
    }

    public boolean isIntakeInPos()
    {
        return intakeInPosition;
    }

    public void softLimitOverideDisabled()
    {
        io.intakeConfigureSoftLimitOverride(false);
    }
    public void softLimitOverideEnabled() 
    {
        io.intakeConfigureSoftLimitOverride(true);
    }
    
    public static CatzIntakeSubsystem getInstance()
    {

        if(instance == null)
        {

            instance = new CatzIntakeSubsystem();

        }

        return instance;
    
    }
    


}
