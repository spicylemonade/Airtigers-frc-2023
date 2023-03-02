import argparse
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation
import pdb
x1=0
x2=0

def main(Kdx,Kpt,Kdt):
    # Define Robot Parameters
    mass = 125 * (1/2.20462); # Mass of the robot in Kg
    l_robot = 3 * (12)*(2.54/100); # Length of the robot in meters
    w_robot = 2 * (12)*(2.54/100); # Width of the robot in meters

    # Define Charge Station Parameters
    h_charge = 12.5 * (2.54/100); # Height of center of the charge station in meters
    w_charge = 8 * (12)*(2.54/100); # Width of the charge station in meters
    l_charge = 4 * (12)*(2.54/100); # Length of the charge station in meters
    
    # Define Initial conditions
    x_IC = l_charge;            # Initial robot position on the charge station in meters
    xDot_IC = 0;                    # Initial robot velocity in m/s
    theta_IC = -15 * (2*math.pi/360); # Initial orientation of the charging station
    thetaDot_IC = 0;            # Initial angular velocity of the charging station
    IC = np.array([x_IC,xDot_IC,theta_IC,thetaDot_IC]);

    # Define the simulation duration
    startTime = 0; # This is the starting time of the simulation (seconds)
    endTime = 20;  # This is the end time of the simulation
    dt = 0.005;     # This is the time step difference
    time = np.arange(startTime,endTime+dt,dt)
#   pdb.set_trace()

    # Simulate the dynamics
    x,xDot,theta,thetaDot,u = balance(IC,time,mass,dt,l_robot,w_robot,l_charge,w_charge,h_charge,Kdx,Kpt,Kdt)
    

    # Plotting
    # fig,ax = plt.subplots(1,1);
    # plotFigs(x,xDot,theta,thetaDot,time,l_charge,h_charge,w_charge,l_robot,w_robot,0,ax);
    # plt.show(block=False); 
    
    # fig,ax = plt.subplots(1,1);
    # plt.plot(time,theta,label='Ramp Orientation');
    # plt.plot(time,x,label='Robot Position');plt.grid();
    # plt.xlabel("Time: (sec)");
    # plt.legend(framealpha=1, frameon=True, loc=2);
    # plt.show(block=False);
    
    
    # fig,ax = plt.subplots(1,1);
    # #pdb.set_trace();
    # skip = 20;
    # ani=FuncAnimation(fig,animate,frames = int(np.floor(len(time)/skip)), interval=100, blit=False, repeat=False,fargs=(ax,x,xDot,theta,thetaDot,time,l_charge,h_charge,w_charge,l_robot,w_robot,skip));
    # ani.save("movie_1.gif",dpi=150,writer=animation.PillowWriter(fps=30))
    # plt.show();
    
    
def animate(i,ax,x,xDot,theta,thetaDot,time,l_charge,h_charge,w_charge,l_robot,w_robot,skip):
    ax.clear()
    plotFigs(x,xDot,theta,thetaDot,time,l_charge,h_charge,w_charge,l_robot,w_robot,int(skip*i),ax)


def plotFigs(x,xDot,theta,thetaDot,time,l_charge,h_charge,w_charge,l_robot,w_robot,ind,ax1):
    #plt.close('all')
    # Figure and Axes

    # Plot triangle
    plotTriangle(h_charge,ax1,ind);
    # Plot Ground
    plotGround(w_charge,ind)
    # Plot Ramp
    plotRamp(h_charge,l_charge,theta,ax1,ind)
    # Plot Robot
    plotRobot(h_charge,x,theta,w_robot,ind)

    # Legend
#   plt.legend("Ramp: "+str(round(theta[ind]*(180/math.pi),2))+" deg")  

    # Axes Limits
    ax1.axis('equal')
    plt.xlim([-w_charge,w_charge])
    plt.ylim([0,3])

    # Title
    plt.title("Robot is the Red Dot ")
#   text_with_autofit(ax1, ["Time: "+str(round(time[ind],2))+" sec"], (0.5, 0.5), 0.4, 0.4, show_rect=False)
    txt = plt.text(-1,2.5,["Time: "+"{:2.2f}".format(time[ind])+" sec"]);
#   txt._get_wrap_line_width = lambda : 190.
    txt1=plt.text(-1,2,[" Ramp: "+"{:2.2f}".format(theta[ind]*(180/math.pi))+" deg"]);
#   txt1._get_wrap_line_width = lambda : 90.
    
def plotRobot(h,x,theta,w,ind):
    robot_h = 0.5;
    rbo = np.array([x[ind]*math.cos(theta[ind]),x[ind]*math.sin(theta[ind])+h]);
    plt.plot(rbo[0],rbo[1],'ro')

def plotTriangle(h_charge,ax1,ind):
    # Triangle 1
    x=np.array([-0.1,0.1,0,-0.1])
    y=np.array([0 , 0,h_charge,0])
    
    # Plot Triangle 1 
    plt.plot(x,y,color=[0/255,176/255,80/255])

def plotRamp(h,l_charge,theta,ax1,ind):
    x = np.array([-l_charge*math.cos(theta[ind]),l_charge*math.cos(theta[ind])])
    y = h+np.array([-l_charge*math.sin(theta[ind]),l_charge*math.sin(theta[ind])])
    plt.plot(x,y,'-',color=[0/255,0/255,0/255])
    #plt.plot((-l_charge*np.cos(theta[ind])),(h-l_charge*np.sin(theta[ind])),'go')
    #plt.plot((l_charge*np.cos(theta[ind])),(h+l_charge*np.sin(theta[ind])),'go')

def plotGround(w_charge,ind):
    x = np.array([-w_charge,w_charge]);
    y = np.array([0,0]);
    plt.plot(x,y,'-',color=[0/255,176/255,80/255])

def balance(IC,time,m,dt,l_robot,w_robot,l_charge,w_charge,h_charge,KDx,KPt,KDt):
    g = -9.81; # gravity
    Cd = 50;
    # Pre-define arrays
    x = np.zeros(len(time));
    xDot = np.zeros(len(time));
    theta = np.zeros(len(time));
    thetaDot = np.zeros(len(time));
    u = np.zeros(len(time));

    # Assign Initial Conditions
    x[0] = IC[0];
    xDot[0] = IC[1];
    theta[0] = IC[2];
    thetaDot[0] = IC[3];

    # Controller
    KPx = 0; 
#   KDx = -60;
#   KPt = 15;
#   KDt = 95;
    #pdb.set_trace()
    """ Simulate using Euler's Method """
    for n in range(len(time)-1):
        leftRamp = (h_charge-l_charge*np.sin(theta[n]));
        rightRamp = (h_charge+l_charge*np.sin(theta[n]));
        # Normal Force
        N = m*(xDot[n]*thetaDot[n]+g*np.cos(theta[n])*(1-x[n]**2/Inertia(x[n],m,l_robot,w_robot)));
        # Drive Force
        u[n] = 1*(-m*g*np.sin(theta[n]) - (m)*x[n]*thetaDot[n]**2) + KDx*xDot[n] + KPt*np.sin(theta[n]) + KDt*thetaDot[n];
        # KDx: how aggressive is my velocity to balance
        # KPt: how to respond to ramp angle
        # KDt: how to respond to ramp angular velocity
        
        if((leftRamp<=0 and thetaDot[n]>0) or (rightRamp<=0) and thetaDot[n]<0):
            theta[n+1]    = theta[n];#pdb.set_trace();
        else:
            theta[n+1]    = theta[n] + dt*(thetaDot[n])
        
        if((x[n]<=(-l_charge*np.cos(theta[n])) and xDot[n]<0 ) or (x[n]>=(l_charge*np.cos(theta[n])) and xDot[n]>0)):
            x[n+1] = x[n]
        else:
            x[n+1]        = x[n] + 1*dt*(xDot[n])
        
        #x[n+1] = x[n]+dt*xDot[n];
        xDot[n+1] = xDot[n] + 1*dt*( 1/m*( u[n] + m*g*np.sin(theta[n])) + x[n]*thetaDot[n]**2  );
        #theta[n+1] = theta[n] + dt*thetaDot[n];
        thetaDot[n+1] = thetaDot[n] + dt*( 1/Inertia(x[n],m,l_robot,w_robot)*(m*g*x[n]*np.cos(theta[n])) - Cd*thetaDot[n]);
        insec = 0.005*n
        timesec = 0.005*len(time)
        if((insec+10<=timesec) and insec <=20):
            if((x[n+int((10)/0.005)]-theta[n]<=0.1)and(x[n+int((10)/0.005)]-theta[n]>=-0.1)):
                print("dx: ",KDx,"pt: ",KPt,"dt: ",KDt)
        #pdb.set_trace();
    
    return x,xDot,theta,thetaDot,u


def Inertia(x,m,l,w):
    I = 1/12*m*(l**2+w**2)+m*x**2;
    return I;
    
for Kdx in range(45,100):
    for Kpt in range(45,100):
        for Kdt in range(45,100):
            main(Kdx*-1,Kpt,Kdt)
            print("next")
# if __name__ == '__main__':
#   parser = argparse.ArgumentParser(description='Apply and simulate a controller to autonomously balance on the charge station')
#   parser.add_argument('--KDx', type=float, default=-60, metavar='KDx',
#                         help='Control gain for the robots velocity')
#   parser.add_argument('--KPt', type=float, default=-15, metavar='KPt',
#                         help='Control gain for the robots orientation')
#   parser.add_argument('--KDt', type=float, default=-95, metavar='KDt',
#                         help='Control gain for the robots angular velocity')
#   args = parser.parse_args()
#   print(args)


# main();

