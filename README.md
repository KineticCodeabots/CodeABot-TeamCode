# Codeabot TeamCode

## TeamCode tructure

`CodeabotCommon.java`: This class defines common enums, such as `Alliance` (BLUE, RED) and `StartingLocation` (AUDIENCE, BACKSTAGE).

**`opmodes`** Robot OpModes

- `AutoBase.java`: This abstract class provides the base functionality for autonomous op modes.
- `AutoRobot.java`: This class extends the Robot class and provides specific functionality for autonomous operations, such as driving straight or turning to a specific heading.
- `RobotTele.java`: This is the tele opmode

**`opmodes.autos`**
Auto OpModes for different starting states.

**`vision`** Vision processing

- `TeamPropDetermination.java`: This class is a `VisionProcessor` and is used to determine the position of a team prop based on color thresholds.
