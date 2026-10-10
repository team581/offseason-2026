# Robot diagnostic regression

Run: 8e9b996f-ff96-4096-a009-f4ec7a8ebb74

Overall: **FAILED**

Configuration: StraightLineConfig[distance=2.0, direction=0.0, maxVelocity=1.0, maxAcceleration=0.75, timeout=10.0, positionTolerance=0.1, headingTolerance=3.0, stoppedVelocity=0.1, settleSeconds=0.25, crossTrackTolerance=0.2]; voltage=12.0; heading=0.0

| Step | Result | Reason |
|---|---|---|
| DriveLeg1 | FAILED | Commanded vector acceleration exceeded limit; command acceleration=25.167362963708563 m/s²; measured window acceleration=0.0 m/s²; configured acceleration limit=0.75 m/s² |
| DriveLeg2 | BLOCKED | Earlier diagnostic failed or run was interrupted |
| DriveLeg3 | BLOCKED | Earlier diagnostic failed or run was interrupted |
| DriveLeg4 | BLOCKED | Earlier diagnostic failed or run was interrupted |
