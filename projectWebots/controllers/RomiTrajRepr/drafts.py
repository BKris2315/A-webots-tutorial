import numpy as np
encs = np.loadtxt("trajectory_velocity.txt", unpack = True)
print(encs[0, :])
dt = [encs[0, n]-encs[0, n-1] for n in range(1,len(encs[0]))]
adt = np.average(dt)
print(adt)

def get_vels():
    pose = np.loadtxt("out.txt", unpack = True)
    o = pose[0,:]
    x = pose[1,:]/100
    y = pose[2,:]/100
    n = len(x)
    do = [o[n]-o[n-1] for n in range(1,len(x))]
    dx = [x[n]-x[n-1] for n in range(1,len(x))]
    dy = [y[n]-y[n-1] for n in range(1,len(y))]
    vrs = []
    vls = []
    for i in range(n-1):
        vrs.append(dx[i]*np.cos(o[i])-0.1415*0.5*do[i])
        vls.append(dx[i]*np.cos(o[i])+0.1415*0.5*do[i])
    return np.array(vrs), np.array(vls), n

vx, vy, n = get_vels()

print(vx[0:50], n)
print(0.25088463*0.0346*encs[1,0:50]*np.pi/360)