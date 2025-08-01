## CHEATS:

### STANDARD SEQUENCE FOR UNITREE ROBOT
```bash
1.- Damp → joints relaxed so you can align the feet.
```
```bash
2.- Stand-up (FSM 4) → robot’s internal routine extends legs part-way.
```
```bash
3.- Increment SetStandHeight in small steps until mode flips 2 → 0.
This is the height at which the robot will stand.
```
```bash
4.- BalanceStand(0) (or SetBalanceMode(0)).
This engages the balance mode, allowing the robot to adjust its posture.
```
```bash
5.- Re-send the final SetStandHeight now that balance is engaged.
```
```bash
6.- Optionally enable continuous gait (SetBalanceMode(1)) and finally Start (FSM 200) to walk.
```