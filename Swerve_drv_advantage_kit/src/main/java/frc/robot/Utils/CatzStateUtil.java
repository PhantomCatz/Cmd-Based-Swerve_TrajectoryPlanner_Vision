/***
 * CatzStateUtil
 * @version 1.0
 * @author Kynam Lenghiem
 * 
 * This class is where state machine enums are defined to give commands instructions
 * on what state to move to.
 **/
package frc.robot.Utils;

public class CatzStateUtil 
{
public static SetMechanismState currentMechState = null;

  public static enum SetMechanismState {

    SCORE_HIGH,
    SCORE_MID,
    SCORE_LOW,
    PICKUP_GROUND,
    PICKUP_SINGLE,
    PICKUP_DOUBLE,
    STOW

  }  

  public static GamePieceState currentGamePieceState = null;

  public static enum GamePieceState {
    
    CONE,
    CUBE,
    NONE
    
  }  

  public static void newGamePieceState(GamePieceState newGamePieceState)
  {
    currentGamePieceState = newGamePieceState;
  }

  
  public static ElevatorState currentElevatorState = null;
  public static enum ElevatorState {

    MANUAL,
    SET_STATE,
    FINISHED
  }

  public static ArmState currentArmState = null;
  public static enum ArmState {

    MANUAL,
    SET_STATE,
    FINISHED
  }

  public static IntakeState currentIntakeState = null;
  public static enum IntakeState {

    MANUAL,
    SET_STATE,
    FINISHED
  }
}
