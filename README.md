# ee4308-proj2

When setting up this workspace for the first time, follow these steps: 
1. Ensure you're in the workspace by running `cd ~/ee4308-proj2` 
2. Run `chmod +x *.sh` 
3. Then run `./clean_make.sh` 

To start the ROS simulation,
1. `./bringup.sh` 
2. `./run.sh` 

Other important details:
- to use the calculated variables, set `use_ground_truth` in `hector.yaml` to be `false`.
- the 

Implementation of various components:
- EKF Prediction for x, y, z, & a (yaw) degrees of freedom → in `cbImu` (`motion.cpp`)
- EKF Correction
  * GPS → in `cpGps` (`motion.cpp`)
  * Magnetometer → in `cbMagnet` (`motion.cpp`)
  * Barometer/altimeter → in `cbBaro` (`motion.cpp`)
  * Sonar → in `cbSonar` (`motion.cpp`)
