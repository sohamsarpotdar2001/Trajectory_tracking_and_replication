import matplotlib.pyplot as plt

m=50
b=30 
vn=0
t=0
dt=0.15
ref=150
e0=ref-vn
ei=0
a=0

time = []
velocity = []
error = []
rlist = []

kp = 50
ki = 18
kd = -15

while t < 30:

    if(abs(0.9*ref - vn )<=50)and t<=10:
        print("time=",t)    
    time.append(t)
    velocity.append(vn)
    rlist.append(ref)
        
    e=ref-vn
    error.append(e)
    ed=(e-e0)/dt
    ei=ei+(e*dt)
    e0=e
    
    u=(e*kp)+(ed*kd)+(ei*ki)
    a=(u-b*vn)/m
    vn=a*dt+vn
    t=t+dt
    
print("min error is ",((min(error))))
plt.plot(time,rlist)

plt.plot(time,error)
plt.plot(time,velocity)
plt.xlabel("Time ")
plt.ylabel("Velocity & error")

plt.show()