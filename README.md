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

# Camera configuration

If you are running more than one of the Arducam cameras on the same host, you will need to change the camera name on the device and/or give it a unique serial number. You can download the (Windows only) configuration tool from <https://docs.arducam.com/UVC-Camera/Serial-Number-Tool-Guide/#software>. The program gets confused if you have non-camera USB devices plugged in (like a mouse).

# Camera Calibration

There are a couple ways to do this as documented on the PhotonVision site. 

# Plan to Record
  1. Coprocessors (OrangePi 5, Raspberry Pi 5, Raspberry pi 4, Windows i7-770)
  2. Cameras: Arducam OV9281, Arducam OV2311, See3CAM_24CUG, Pi camera (cable)
  3. Resolutions 1280x720, 800x600, 640x480
  4. record FPS, latency, power usage, accuracy


  # Hints, Clues, Reminders

   PhotonVision logs and config are in `/opt/photonvision/photonvision_config/`