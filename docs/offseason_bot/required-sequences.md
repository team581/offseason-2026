# Required Sequences

## Shot Verification

- Drive to known locations and shoot X number of balls
- Have constrained shoot on the move patterns
  - Centered on goal, move straight back
  - Side to side Y feet from goal
  - Circular path to the left/right of the goal

This should validate vision

## Quick Shot Retuning

1. Create regression equation.
   - Use shot table (distance → angle/velocity) to create `Angle(distance)` and `Velocity(distance)` equations.
   - Input equation into code to use.
2. Modify regression equation when needed.
   - To shift shot forward X inches, `Angle(distance)` now becomes `Angle(distance - X)`, and same for `Velocity`.
