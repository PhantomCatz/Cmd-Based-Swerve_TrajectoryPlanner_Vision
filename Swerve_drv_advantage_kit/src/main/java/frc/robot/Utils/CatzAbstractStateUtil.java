/***
 * CatzStateUtil
 * @version 1.0
 * @author Kynam Lenghiem
 * 
 * This class is where state machine enums are defined to give commands instructions
 * on what state to move to.
 **/
package frc.robot.Utils;

import frc.robot.RobotContainer;
import frc.robot.RobotContainer.GamePiece;

public class CatzAbstractStateUtil 
{
public static SetAbstractMechanismState currentAbstractMechState = null;

  public static enum SetAbstractMechanismState {
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

  public static void newGamePieceState(GamePieceState newGamePieceState) {
    currentGamePieceState = newGamePieceState;

    //LED logic
    if(currentGamePieceState == GamePieceState.CONE) {
      RobotContainer.currentGamePiece = GamePiece.Cone;
    } 
    else if(currentGamePieceState == GamePieceState.CUBE) {
      RobotContainer.currentGamePiece = GamePiece.Cube;
    }
    else {
      RobotContainer.currentGamePiece = GamePiece.None;
    }
  }
}
