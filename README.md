# PhotonVision Benchmarking

A setup to read PhotonVision data from NetworkTables from a vision co-processor. 

Goals of this tool are:
1. Test and tune the camera calibration
2. Measure error and noise.
3. Benchmark FPS, latency, and jitter as seen by the robot
4. Create camera configuration

A work in progress. (just started actually)

# Setup

Run this robot project in simulation. Configure the host's IP address to be 10.TE.ME.2 so that the PhotonVision coprocessor will connect to it. The computer running this robot code must be on the same 10.TE.ME.x network as the vision coprocessor. 