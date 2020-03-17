import airsim
import dronesim
import msvcrt

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

uav = dronesim.VehicleMotion(client)
uav.start()
uav.run()
print('run')
while True:
    key = msvcrt.getch()
    if key==b'w':
        uav.flyCmd("forward","fast")
    if key==b's':
        uav.flyCmd("backward","fast")
    if key==b'a':
        uav.flyCmd("moveleft","fast")
    if key==b'd':
        uav.flyCmd("moveright","fast")
    if key==b'j':
        uav.flyCmd("turnleft","fast")
    if key==b'l':
        uav.flyCmd("turnright","fast")
    if key==b'i':
        uav.flyCmd("up","fast")
        print('c')
    if key==b'k':
        uav.flyCmd("down","fast")
    if key==b'q':
        uav.flyCmd("stop")


