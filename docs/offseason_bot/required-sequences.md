# Required Sequences

## Shot Verification

- Before running, record the measured test locations, number of balls and attempts, path distances/speeds, and acceptable scoring/aiming results using the [behavior verification procedure](verification.md).
- Drive to the agreed locations and shoot the recorded number of balls.
- Have constrained shoot on the move patterns
  - Centered on goal, move straight back
  - Side to side at the recorded distance from the goal
  - Circular path to the left/right of the goal

Compare logged aiming/localization behavior with independently observed shot results. Record failures and evidence in the existing task; passing a shot sequence alone does not establish complete vision-system qualification.

## Quick Shot Retuning

1. Create regression equation.
   - Use shot table (distance → angle/velocity) to create `Angle(distance)` and `Velocity(distance)` equations.
   - Input equation into code to use.
2. Modify regression equation when needed.
   - To shift shot forward X inches, `Angle(distance)` now becomes `Angle(distance - X)`, and same for `Velocity`.
