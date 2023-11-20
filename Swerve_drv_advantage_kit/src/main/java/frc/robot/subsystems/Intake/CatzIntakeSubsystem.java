// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.CatzConstants;
import frc.robot.Utils.CatzManipulatorPositions;
import frc.robot.Utils.CatzSharedDataUtil;
import frc.robot.Utils.CatzAbstractStateUtil;

public class CatzIntakeSubsystem extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public static CatzIntakeSubsystem instance;

    // ----------------------------------------------------------------------------------------------
    //
    // Roller Control Constants
    //
    // ----------------------------------------------------------------------------------------------

    private final double ROLLERS_PWR_CUBE_IN = -0.8;
    private final double ROLLERS_PWR_CONE_IN = 1.0; // TBD decide pwrs for all cube cone scoring rollers

    private final double ROLLERS_PWR_CUBE_OUT = 1.0;
    private final double ROLLERS_PWR_CONE_OUT = -0.5;

    private PIDController intakePID;

    private double targetPositionDeg = CatzConstants.IntakeConstants.STOW_ENC_POS;

    private double prevTargetPwr = 0.0;

    private double currentPosition = -999.0;
    private double positionError = -999.0;
    public double prevCurrentPosition = -999.0;

    private boolean intakeInPosition = false;

    private final double INTAKE_POS_ERROR_THRESHOLD_DEG = 5.0;
    private final double PID_FINE_GROSS_THRESHOLD_DEG = 17.0;

    private int numConsectSamples = 0;
    private double m_fullManualPwr = 0.0;

    private CatzManipulatorPositions m_targetPos;

    private CatzIntakeSubsystem() {
        switch (CatzConstants.currentMode) {
            case REAL:
                io = new IntakeIOReal();
                break;
            case SIM:
                io = null; // new IntakeIOSim();
                break;
            default:
                io = new IntakeIOReal() {
                };
                break;
        }
        intakePID = new PIDController(CatzConstants.IntakeConstants.GROSS_kP, CatzConstants.IntakeConstants.GROSS_kI,
                CatzConstants.IntakeConstants.GROSS_kD);

    }

    @Override
    public void periodic() {
        // perform input updates
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Intake", inputs);

        if (DriverStation.isDisabled()) {
            io.wristSetPercentOuputIO(0.0);
            m_targetPos = null;
        } else if (m_targetPos != null) {
            targetPositionDeg = m_targetPos.getWristAngleDeg();
            // ----------------------------------------------------------------------------------
            // Chk if at final position
            // ----------------------------------------------------------------------------------
            currentPosition = inputs.wristPosEnc / CatzConstants.IntakeConstants.WRIST_CNTS_PER_DEGREE;
            positionError = currentPosition - targetPositionDeg;

            if ((Math.abs(positionError) <= INTAKE_POS_ERROR_THRESHOLD_DEG)) {
                numConsectSamples++;
                if (numConsectSamples >= 1) {
                    intakeInPosition = true;
                }
            } else {
                numConsectSamples = 0;
            }

            if (Math.abs(positionError) >= PID_FINE_GROSS_THRESHOLD_DEG) {
                intakePID.setP(CatzConstants.IntakeConstants.GROSS_kP);
                intakePID.setI(CatzConstants.IntakeConstants.GROSS_kI);
                intakePID.setD(CatzConstants.IntakeConstants.GROSS_kD);
            } else if (Math.abs(positionError) < PID_FINE_GROSS_THRESHOLD_DEG) {
                intakePID.setP(CatzConstants.IntakeConstants.FINE_kP);
                intakePID.setI(CatzConstants.IntakeConstants.FINE_kI);
                intakePID.setD(CatzConstants.IntakeConstants.FINE_kD);
            }

            double pidPower = intakePID.calculate(currentPosition, targetPositionDeg);
            double ffPower = calculateGravityFF();
            double targetPower = pidPower + ffPower;

            // -------------------------------------------------------------
            // checking if we did not get updated position value(Sampling Issue).
            // If no change in position, this give invalid target power(kD issue).
            // Therefore, go with prev targetPower Value.
            // -------------------------------------------------------------------
            if (prevCurrentPosition == currentPosition) {
                targetPower = prevTargetPwr;
            }

            // ----------------------------------------------------------------------------------
            // If we are going to Stow Position & have passed the power cutoff angle, set
            // power to 0, otherwise calculate new motor power based on position error and
            // current angle
            // ----------------------------------------------------------------------------------
            if (targetPositionDeg == CatzConstants.IntakeConstants.STOW_ENC_POS
                    && currentPosition > CatzConstants.IntakeConstants.STOW_CUTOFF) {
                targetPower = 0.0;
            }
            io.wristSetPercentOuputIO(targetPower);
            prevCurrentPosition = currentPosition;
            prevTargetPwr = targetPower;
        } else {
            io.wristSetPercentOuputIO(m_fullManualPwr);
        }
    }

    public void cmdUpdateIntake(CatzManipulatorPositions targetPos) {
        this.m_targetPos = targetPos;
    }

    public void manualHoldingFunction(double wristPwr) {
        if(wristPwr > 0) {
          targetPositionDeg = Math.min((targetPositionDeg + wristPwr * CatzConstants.IntakeConstants.MANUAL_HOLD_STEP_SIZE), CatzConstants.IntakeConstants.SOFT_LIMIT_FORWARD);
        }
        else {
          targetPositionDeg = Math.max((targetPositionDeg + wristPwr * CatzConstants.IntakeConstants.MANUAL_HOLD_STEP_SIZE), CatzConstants.IntakeConstants.SOFT_LIMIT_REVERSE);
        }
        prevCurrentPosition = -prevCurrentPosition; //intialize for first time through thread loop, that checks stale position values
        m_targetPos = new CatzManipulatorPositions(CatzSharedDataUtil.sharedElevatorEncCnts, CatzSharedDataUtil.sharedArmEncCnts, targetPositionDeg);
    }

    public void wristFullManual(double fullManualPwr) {
        m_fullManualPwr = fullManualPwr;
        m_targetPos = null;
    }

    /*----------------------------------------------------------------------------------------------
    *
    *  Utilities - PID 
    *
    *---------------------------------------------------------------------------------------------*/
    public void resetPID() {
        intakePID.reset();
    }

    /*----------------------------------------------------------------------------------------------
    *
    *  Utilities - Rollers
    *
    *---------------------------------------------------------------------------------------------*/
    public void intakeRollerFunctionIN() {
        if (CatzAbstractStateUtil.currentGamePieceState == CatzAbstractStateUtil.GamePieceState.CUBE) {
            rollersInCube();
        } else {
            rollersInCone();
        }
    }

    public void intakeRollerFunctionOUT() {
        if (CatzAbstractStateUtil.currentGamePieceState == CatzAbstractStateUtil.GamePieceState.CUBE) {
            rollersOutCube();
        } else {
            rollersOutCone();
        }
    }

    public void intakeRollersOff() {
        io.rollersOffIO();
    }

    private void rollersInCube() {
        io.rollersOnIO(ROLLERS_PWR_CUBE_IN);
    }

    private void rollersOutCube() {
        io.rollersOnIO(ROLLERS_PWR_CUBE_OUT);
    }

    private void rollersInCone() {
        io.rollersOnIO(ROLLERS_PWR_CONE_IN);
    }

    private void rollersOutCone() {
        io.rollersOnIO(ROLLERS_PWR_CONE_OUT);
    }

    /*----------------------------------------------------------------------------------------------
    *
    *  Utilities - Wrist
    *
    *---------------------------------------------------------------------------------------------*/
    public double calcWristAngle() {
        double wristAngle = ((inputs.wristPosEnc / CatzConstants.IntakeConstants.WRIST_CNTS_PER_DEGREE));
        return wristAngle;
    }

    private double getWristPosition() {
        return inputs.wristPosEnc;
    }

    public double calculateGravityFF() {
        double radians = Math.toRadians(calcWristAngle() - CatzConstants.IntakeConstants.CENTER_OF_MASS_OFFSET_DEG);
        double cosineScalar = Math.cos(radians);

        return CatzConstants.IntakeConstants.MAX_GRAVITY_FF * cosineScalar;
    }

    public double intakeWristTemp() {
        return inputs.wristTemp;
    }

    public void shuffleboardIntake() {

    }

    public boolean isIntakeInPos() {
        return intakeInPosition;
    }

    public void setisIntakeInPos(boolean isEnabled) {
        intakeInPosition = isEnabled;
    }

    public void softLimitOverideDisabled() {
        io.intakeConfigureSoftLimitOverride(false);
    }

    public void softLimitOverideEnabled() {
        io.intakeConfigureSoftLimitOverride(true);
    }

    // Singleton implementation for instatiating subssytems(Every refrence to this
    // method should be static)
    public static CatzIntakeSubsystem getInstance() {
        if (instance == null) {
            instance = new CatzIntakeSubsystem();
        }
        return instance;
    }
}
