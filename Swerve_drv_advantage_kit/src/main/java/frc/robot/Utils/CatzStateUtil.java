package frc.robot.Utils;

public class CatzStateUtil 
{
public static MechanismState currentMechState;

  public static enum MechanismState {

    ScoreHigh,
    ScoreMid,
    ScoreLow,
    PickupGround,
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

  public static void newGamePieceState(GamePieceState newGamePieceState)
  {
    currentGamePieceState = newGamePieceState;
  }

  
  public static ElevatorState currentElevatorState;
  public static enum ElevatorState {

    MANUAL,
    SET_STATE
  }

  public static ArmState currentArmState;
  public static enum ArmState {

    MANUAL,
    SET_STATE
  }

  public static IntakeState currentIntakeState;
  public static enum IntakeState {

    MANUAL,
    SET_STATE
  }
}
