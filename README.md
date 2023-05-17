# DroneKit-Code
Python Scripts for SAE INDIA Drone

To Open Simulation:

sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console

gazebo --verbose worlds/iris_arducopter_runway.world

mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551

