# What the rover thought - test1_failed in open sea.bin (280 s)

   0.0-   7.8 s  ready (8 s)
   8.0-  15.3 s  Leg 1 (B facade, wall on the left) (7 s, 0.0 -> 1.4 m)
      wall on the left: 1.59-1.60 m (median 1.60)
         8.3 s  saw a pillar on the right
  15.5-  17.8 s  Waiting (something in front) (2 s)
        15.5 s  something in front -> waiting
  18.0-  31.8 s  Leg 1 (B facade, wall on the left) (14 s, 1.5 -> 4.5 m)
      wall on the left: 1.60-1.61 m (median 1.60)
  32.0-  39.0 s  Waiting (something in front) (7 s)
        32.0 s  something in front -> waiting
  39.3-  65.5 s  Leg 1 (B facade, wall on the left) (26 s, 4.5 -> 10.1 m)
      wall on the left: 1.60-1.62 m (median 1.61)
        39.5 s  saw a pillar on the right
        53.0 s  saw a pillar on the right
        57.0 s  the wall on the left disappeared (gap / end)
        64.0 s  saw a pillar on the right
  65.8-  68.0 s  PAUSED (2 s)
        65.8 s  paused (STOP / phone / failsafe)
  68.3- 118.8 s  Leg 1 (B facade, wall on the left) (51 s, 10.2 -> 21.4 m)
      wall on the left: 1.47-1.57 m (median 1.54)
        68.3 s  resumed
        77.3 s  saw a pillar on the right
        92.0 s  saw a pillar on the right
        93.0 s  found a wall on the left again
       107.3 s  saw a pillar on the right
       115.5 s  saw a pillar on the right
 119.0- 124.0 s  Turning (5 s)
       119.0 s  found the turn landmark (last pillar) -> turn right
       123.0 s  turn stalled -> tried the next way
 124.3- 125.0 s  Checking the turn (1 s)
 125.3- 129.8 s  Open sea (4 s, 0.0 -> 1.1 m)
      references: the B wall behind 100%
      wanted heading moved -23.1 .. -22.2 deg; heading error median 1.3 deg
       125.3 s  turn finished
       125.8 s  started using the B wall behind
 130.0- 132.2 s  Waiting (something in front) (2 s)
       130.0 s  something in front -> waiting
 132.5- 135.8 s  Open sea (3 s, 1.3 -> 2.0 m)
      references: the B wall behind 100%
      wanted heading moved -23.4 .. -22.9 deg; heading error median 0.3 deg
 136.0- 138.3 s  Waiting (something in front) (2 s)
       136.0 s  something in front -> waiting
 138.5- 146.0 s  Open sea (8 s, 2.1 -> 3.8 m)
      references: the B wall behind 100%
      wanted heading moved -24.6 .. -23.2 deg; heading error median 0.5 deg
 146.3- 148.5 s  Waiting (something in front) (2 s)
       146.3 s  something in front -> waiting
 148.8- 166.5 s  Open sea (18 s, 4.0 -> 7.8 m)
      references: the B wall behind 80%, gyro only 19%
      wanted heading moved -24.5 .. -21.8 deg; heading error median 0.6 deg
 166.8- 168.0 s  PAUSED (1 s)
       166.8 s  paused (STOP / phone / failsafe)
 168.3- 189.5 s  Open sea (21 s, 7.9 -> 12.2 m)
      references: gyro only 55%, the B wall behind 44%
      wanted heading moved -25.1 .. -21.0 deg; heading error median 0.8 deg
       168.3 s  resumed
 189.8- 191.0 s  PAUSED (1 s)
       189.8 s  paused (STOP / phone / failsafe)
 191.3- 224.8 s  Open sea (34 s, 12.3 -> 19.1 m)
      references: the tree/post row on the left 50%, gyro only 46%, the B wall behind 2%
      tree/post row distance 6.53-6.93 m (median 6.82) -> it thought it was on the strip when this was near the setting
      wanted heading moved -24.4 .. -16.3 deg; heading error median 1.2 deg
       191.3 s  resumed
       196.5 s  found the tree/post row on the left
 225.0- 226.5 s  PAUSED (2 s)
       225.0 s  paused (STOP / phone / failsafe)
 226.8- 261.8 s  Open sea (35 s, 19.2 -> 26.6 m)
      references: gyro only 51%, the tree/post row on the left 47%, the building front on the left 1%
      tree/post row distance 6.61-6.82 m (median 6.72) -> it thought it was on the strip when this was near the setting
      wanted heading moved -29.4 .. -20.8 deg; heading error median 0.7 deg
       226.8 s  resumed
 262.0- 262.0 s  Waiting (something in front) (0 s)
       262.0 s  something in front -> waiting
 262.3- 263.5 s  Turning (1 s)
       262.3 s  end wall ahead at the stop distance -> turn right
 263.8- 264.5 s  Checking the turn (1 s)
 264.8- 265.0 s  Turning (0 s)
 265.3- 266.0 s  Checking the turn (1 s)
 266.2- 266.5 s  Turning (0 s)
 266.8- 267.5 s  Checking the turn (1 s)
 267.8- 272.0 s  Leg 3 (arcade, wall on the left) (4 s, 0.0 -> 0.7 m)
      no wall seen on the left - drove on the gyro heading
       267.8 s  turn finished
 272.3- 276.0 s  Waiting (something in front) (4 s)
       272.3 s  something in front -> waiting
 276.3- 280.0 s  PAUSED (4 s)
       276.3 s  paused (STOP / phone / failsafe)

## Problems found
- LiDAR points closer than 35 cm in 15 % of the scans (daylight noise; v8 ignores them)
