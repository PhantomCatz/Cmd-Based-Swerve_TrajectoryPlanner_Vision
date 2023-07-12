package frc.robot.Utils;

public class CatzStateUtil 
{
public static MechanismState currentMechState;

  public static enum MechanismState {

    ScoreHigh,
    ScoreMid,
    ScoreLow,
    PickupLow,
    PickupSingle,
    PickupDouble,
    Stow

  }  

  public static GamePieceState currentGamePieceState;

  public static enum GamePieceState {

    CONE,
    CUBE,
    NONE

  }  
}
