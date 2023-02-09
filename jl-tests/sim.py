# -*- coding: utf-8 -*-
import sys
sys.path.append ('\Simulations')
import numpy as np
import math
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.patches import Rectangle
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation
import pdb

class Robot:
    # Wheels
    wheel_radius =  3 * (0.0254);	 	  	 	 	# Radius of the wheel
    wheel_mass = 0.5*(1/2.20462); 	 	  	 	 	# mass of the wheel
    I_wheel = 1/2*wheel_mass*wheel_radius**2			# Moment of Inertia of the robot wheels
    wheel_friction = 0.1;				 	  	 	# Damping coefficient of the robot wheels

    # Robot Chasis
    base_length = 27 * (0.0254); 	 	 # Length of the robot (INCORRECT NEED TO COMPUTE)
    base_width = 24 * (0.0254); 		 	 # width of the robot  (INCORRECT NEED TO COMPUTE)
    base_height = 4 * (0.0254);	 		 # height of the chassis  (INCORRECT NEED TO COMPUTE)
    robot_mass = 125 * (1/2.20462);  	 # Mass of the robot in Kg
    robot_height = 36 *(0.0254);	 	 	 # height of the robot (wheels not included)  (INCORRECT NEED TO COMPUTE)

    # Arm
    arm_length_min = 25 * (0.0254); 		 # minimum arm length
    arm_length_max = 49 * (0.0254);		 # maximum arm length
    Total_arm_mass = 12 * (0.453592); 	 # mass of the arm   (INCORRECT NEED TO COMPUTE)
    Extended_arm_mass = 4 * (0.453592);	 # mass of the arm   (INCORRECT NEED TO COMPUTE)
    arm_friction = 0.5;					 # rotational friction in the arm
    arm_width = 3 * (0.0254); 			 # width of the arm
    rotate_arm_width = 2 * (0.0254); 	 # width of the rotating arm
    slide_friction = 5;
    I_arm = 0.3; 						 # Inertia of the arm (INCORRECT NEED TO COMPUTE)
    pistonForce = (60*math.pi*(1.5/2)**2)*4.4482189159; # Force of the piston
    maxArmTorque = 166.4;
    # Roller Claw
    claw_mass = 10 * (0.453592); 		 # Mass of the roller claw
    claw_length = 12 * (0.0254);		 	 # Length of the claw
    I_claw = 0.01; 						 # Moment of inertia of the claw (INCORRECT NEED TO COMPUTE)


    # Functions
    def __init__(self):
        self.data = [];

    def eRb(self,th):
        # Rotation matrix from frame B to the inertial frame
        point = np.array([[np.cos(th),-np.sin(th)],[np.sin(th),np.cos(th)]]);
        return point

    def eRc(self,th):
        # Rotation matrix from frame c to the inertial frame
        point = np.array([[np.sin(th),-np.cos(th)],[np.cos(th),np.sin(th)]]);
        return point

    def plotRobot(self,theta,x,y,l1,arm_theta,ax):
        wheel_1 = plt.Circle(( x ,y+self.wheel_radius ), self.wheel_radius ,color=[150/255,150/255,150/255])
        wheel_2 = plt.Circle(( x+self.base_length ,y+self.wheel_radius ), self.wheel_radius ,color=[150/255,150/255,150/255])
        # Plot robot chasis
        ax.add_patch(Rectangle((x,y+self.wheel_radius),self.base_length,self.base_height,facecolor = [50/255,50/255,50/255],angle=0))
        # Plot vertical arm
        ax.add_patch(Rectangle((x+self.base_length/2,y+self.wheel_radius+self.base_height),self.arm_width,self.robot_height,facecolor = [50/255,50/255,50/255],angle=0))
        # Plot Wheels
        ax.add_patch(wheel_1)
        ax.add_patch(wheel_2)
        plt.plot(x+self.wheel_radius*np.cos(theta),y+self.wheel_radius+self.wheel_radius*np.sin(theta),'r.');
        plt.plot(x+self.base_length+self.wheel_radius*np.cos(theta),y+self.wheel_radius+self.wheel_radius*np.sin(theta),'r.');
        # Plot top of vertical arm
        r_b_p = np.array([[x+self.base_length/2+self.arm_width/2],[ y+self.wheel_radius+self.base_height+self.robot_height]]);
        plt.plot(r_b_p[0],r_b_p[1],'r.',markersize=12);
        # Plot rotating arm base
        wristPoint = r_b_p + np.matmul(self.eRb(arm_theta),np.array([[-l1],[0]]));
        plt.plot([r_b_p[0],wristPoint[0]],[r_b_p[1],wristPoint[1]],'k-',linewidth=4);
        plt.plot(wristPoint[0],wristPoint[1],'r.',markersize=12);



class game_field:
    # Assign Field Dimensions
    charge_center = 96.75 * (0.0254);  # Charge station position on the field
    h_charge = 9.125 * (0.0254); 	# Height of center of the charge station in meters
    l_charge = 4 * 12 * 0.0254; # Length of the charge station in meters
    t_charge = 193 *(1/100);    # Total length of charge station
    half_field = 224*(0.0254);  # Half the field length in meters
    grid_depth = 138/100;		# Depth of the gird in meters
    topLevelgrid = 90/100;		# Height of the top level of the grid
    midLevelgrid = 60/100;		# Height of the mid level of the grid
    cube_depth = 17*0.0254;		# cube Depth
    m_ramp = 20*(1/2.20462);		# Mass of the ramp
    I_ramp = 1/12*m_ramp*t_charge/2**2	# Moment of Inertia of the ramp
    K_ramp = 1;						# Ramp stiffness coefficient
    D_ramp = 1;							# Ramp damping coefficient

    # Charge Station position
    left_charge_station = charge_center - t_charge/2
    right_charge_station = charge_center + t_charge/2

    # Grid
    gridStartHeight = 13/100;
    midDepth = 58/100;
    topDepth = 101/100;

    # Functions
    def __init__(self):
        self.data=[];

    def plotField(self,ax,phi,ind):
        x = np.array([0,self.half_field]);
        y = np.array([0,0]);
        plt.plot(x,y,'-',color=[0/255,176/255,80/255],linewidth=2);
        plt.xlim([-self.grid_depth,self.half_field]);
        # Triangle 1
        x=np.array([-0.1,0.1,0,-0.1])+self.charge_center
        y=np.array([0 , 0,self.h_charge,0])
        # Plot Triangle 1
        plt.plot(x,y,color=[0/255,0/255,255/255])
        plt.xlabel('X (m)');
        plt.ylabel('Y (m)');
        ax.add_patch(Rectangle((0,0),-self.grid_depth,self.gridStartHeight,facecolor = 'blue'));
        ax.add_patch(Rectangle((-self.midDepth+0.5*self.cube_depth,0),-self.cube_depth,self.midLevelgrid,facecolor = 'blue'));
        ax.add_patch(Rectangle((-self.topDepth+0.5*self.cube_depth,0),-self.cube_depth,self.topLevelgrid,facecolor = 'blue'));

        x = self.charge_center + np.array([-self.l_charge/2*math.cos(phi[ind]),self.l_charge/2*math.cos(phi[ind])]);
        y = self.h_charge+np.array([-self.l_charge/2*math.sin(phi[ind]),self.l_charge/2*math.sin(phi[ind])]);
        plt.plot(x,y,'-',color=[0/255,0/255,0/255]);
        plt.plot(x,y,'.',color=[0/255,0/255,0/255]);

        tx = self.charge_center + np.array([-self.t_charge/2,self.t_charge/2]);
        ty = np.zeros((2,1));
        plt.plot(tx,ty,'.',color=[0,0,0]);

        LeftlowerRamp_x = np.array([tx[0],x[0]]);
        LeftlowerRamp_y = np.array([0,y[0]]);
        RightlowerRamp_x = np.array([tx[1],x[1]]);
        RightlowerRamp_y = np.array([0,y[1]]);
        plt.plot(LeftlowerRamp_x,LeftlowerRamp_y,'-',color=[0/255,0/255,0/255]);
        plt.plot(RightlowerRamp_x,RightlowerRamp_y,'-',color=[0/255,0/255,0/255]);
        plt.ylim([0,2]);
        ax.axis('equal');
        plt.grid();

def main():
    # Do you want to show plots and animate a gif
    showPlots = True;
    showAnimation = True;

    # Define Checklist
    global scored;
    global mobility;
    global balanced;

    # Simulate Robot during Autonomous
    startTime = 0; # This is the starting time of the simulation (seconds)
    endTime = 15;  # This is the end time of the simulation
    dt = 0.001;     # This is the time step difference
    time = np.arange(startTime,endTime+dt,dt)

    # Initial Conditions
    x_IC = 0.25;		 			    # Initial robot position in meters
    xDot_IC = 0;					 	# Initial robot velocity in m/s
    y_IC = 0; 					 	# Initial robot position in meters
    yDot_IC = 0;					 	# Initial robot velocity in m/s
    phi_IC = 0*(2*math.pi/360);	    # Initial robot orientation
    phiDot_IC = 0;			        # Initial robot angular velocity
    theta_IC = 0 * (2*math.pi/360); # Initial orientation of the robot's wheels
    thetaDot_IC = 0;			 	 	# Initial angular velocity of the robot's wheels
    gamma_IC = 0; 					# Initial charge station orientation
    gammaDot_IC = 0; 				# Initial charge station angular velocity
    sigma_IC = 0*(math.pi/180); 	 	# Initial preRamp Orientation
    sigmaDot_IC = 0;					# Initial reRamp angular Rate
    arm_theta_IC = 80*(math.pi/180);# Initial arm orientation
    arm_thetaDot_IC = 0;	 			# Initial arm angular velocity
    l1_IC = 25 * (0.0254); 			# Initial arm length
    l1Dot_IC = 0; 					# Initial arm velocity

    IC = np.array([x_IC,xDot_IC,y_IC,yDot_IC,phi_IC,phiDot_IC,theta_IC,thetaDot_IC,gamma_IC,gammaDot_IC,sigma_IC,sigmaDot_IC,arm_theta_IC,arm_thetaDot_IC,l1_IC,l1Dot_IC]);

    X = odeint(autonomousCode,IC,time,args=(Robot,game_field));

    # Assign Variables
    x=X[:,0];
    xDot=X[:,1];
    y=X[:,2];
    yDot=X[:,3];
    phi=X[:,4];
    phiDot=X[:,5];
    theta=X[:,6];
    thetaDot=X[:,7];
    gamma=X[:,8];
    gammaDot=X[:,9];
    sigma=X[:,10];
    sigmaDot=X[:,11];
    armTheta=X[:,12];
    armThetaDot=X[:,13];
    l1=X[:,14];
    l1Dot=X[:,15];

    # Plots and Animation
    robot=Robot();
    field=game_field();

    if(showPlots):
        Plots(robot,field,x,xDot,y,yDot,phi,phiDot,theta,thetaDot,gamma,gammaDot,sigma,sigmaDot,armTheta,armThetaDot,l1,l1Dot,time)

    if(showAnimation):
        fig,ax = plt.subplots(1,1);
        # 	frames = 450;
        frames = 50;
        skip = int(len(time)/frames);
        ani=FuncAnimation(fig,animate,frames = int(np.floor(len(time)/skip)), interval=100, blit=False, repeat=False,fargs=(ax,skip,robot,field,theta,x,y,phi,l1,armTheta,time));
        ani.save("autonomous_1.gif",dpi=150,writer=animation.PillowWriter(fps=30))
        plt.show();

def animate(i,ax,skip,robot,field,theta,x,y,phi,l1,arm_theta,time):
    ax.clear();
    ind = i*skip;
    # Axes Limits
    # 	ax.axis('equal');
    field.plotField(ax,phi,ind);
    robot.plotRobot(theta[ind],x[ind],y[ind],l1[ind],arm_theta[ind],ax);
    txt = plt.text(1,1.5,["Time: "+"{:2.2f}".format(time[ind])+" sec"]);

def Plots(robot,field,x,xDot,y,yDot,phi,phiDot,theta,thetaDot,gamma,gammaDot,sigma,sigmaDot,armTheta,armThetaDot,l1,l1Dot,time):
    # Plot Initial Configuration
    fig,ax = plt.subplots(1,1);
    field.plotField(ax,phi,0);
    robot.plotRobot(theta[0],x[0],y[0],l1[0],armTheta[0],ax);
    plt.title('Initial Position on Field')
    plt.show(block=False);
    # 	pdb.set_trace()

    # Plot robot horizontal position
    plt.figure();
    plt.plot(time,x,label='Robot Position');plt.grid();
    plt.xlabel('Time (sec)');
    plt.ylabel('Robot Position');
    plt.legend(framealpha=1, frameon=True, loc=2);
    plt.show(block=False);

    # Plot robot horizontal velocity
    plt.figure();
    plt.plot(time,xDot,label='Robot Velocity'); plt.grid();
    plt.xlabel('Time (sec)');
    plt.ylabel('Robot Velocity');
    plt.legend(framealpha=1, frameon=True, loc=2);
    plt.show(block=False);

    # Plot robot wheel orientation
    plt.figure();
    plt.plot(time,theta,label='Wheel Orientaion'); plt.grid();
    plt.xlabel('Time (sec)');
    plt.ylabel('Wheel Orientaion (rad)');
    plt.legend(framealpha=1, frameon=True, loc=3);
    plt.show(block=False);

    # Plot robot wheel angular velocity
    plt.figure();
    plt.plot(time,thetaDot,label='Wheel Ang Rate'); plt.grid();
    plt.xlabel('Time (sec)');
    plt.ylabel('Wheel Angular Velocity (rad/s)');
    plt.legend(framealpha=1, frameon=True, loc=3);
    plt.show(block=False);

    # Plot ramp orientation
    plt.figure();
    plt.plot(time,phi,label='Ramp Orientaion'); plt.grid();
    plt.plot(time,phiDot,label='Ramp Ang Rate');
    plt.xlabel('Time (sec)');
    plt.legend(framealpha=1, frameon=True, loc=2);
    plt.show(block=False);

    # Plot pre-ramp orientation
    plt.figure();
    plt.plot(time,sigma,label='PreRamp Orientaion'); plt.grid();
    plt.plot(time,sigmaDot,label='PreRamp Ang Rate');
    plt.xlabel('Time (sec)');
    # 	plt.ylabel('Wheel Orientaion (rad)');
    plt.legend(framealpha=1, frameon=True, loc=2);
    plt.show(block=False);

    # Plot robot arm orientation
    plt.figure();
    plt.plot(time,armTheta,label='arm orientation'); plt.grid();
    plt.xlabel('Time (sec)');
    plt.ylabel('Arm Orientation (rad)');
    plt.legend(framealpha=1, frameon=True, loc=2);
    plt.show(block=False);
    # 	pdb.set_trace()

    # Plot robot telescoping arm length
    plt.figure();
    plt.plot(time,l1,label='arm length'); plt.grid();
    plt.xlabel('Time (sec)');
    plt.ylabel('Arm Length (m)');
    plt.plot(time,(25*0.0254)*np.ones((time.shape)),'r--',label='Upper Limit');
    plt.plot(time,(49*0.0254)*np.ones((time.shape)),'r--',label='Lower Limit');
    plt.legend(framealpha=1, frameon=True, loc=2);
    plt.show(block=False);
# 	pdb.set_trace()

def autonomousCode(X,t,robot,game_field):
    global scored;
    g = -9.81; # gravity


    # Assign State Variables
    x,xDot,y,yDot,phi,phiDot,theta,thetaDot,gamma,gammaDot,sigma,sigmaDot,arm_theta,arm_thetaDot,l1,l1Dot = X


    U = Inputs(x,xDot,y,yDot,theta,thetaDot,arm_theta,arm_thetaDot,l1,l1Dot);


    if((x>= game_field.left_charge_station and x<= game_field.right_charge_station)):
        # If robot is at the charge station base from the left side
        y = 1;

    elif(((x+Robot.base_length)>= game_field.left_charge_station and (x+Robot.base_length)<= game_field.right_charge_station)):
        # If robot is at the charge station base from the right side
        y=1;
    elif(x<=0 and xDot<0):
        # If robot is at the grid
        y=1;
    else:
        # If robot is free moving (NOT NEAR OBSTACLES)
        y=1;
    """
    m2 = 10;
    Tp1 = -0.5*m2*t_charge*phiDot**2 + 1/(2*I_ramp)*m2*t_charge*K_ramp*phi*np.tan(phi) - 1/(2*I_ramp)*m2*t_charge*D_ramp*phiDot*np.tan(phi);
    L = 0.2;
    m1 = 2;
    I_smallRamp = 1/12*m1*L**2;
    dsigmadt = sigmaDot;
    ddsigmadt = 0;#1/I_smallRamp*(L*Tp1*np.sin(phi-sigma) - 0.5*L*m1*g*np.cos(sigma) -D_ramp*sigmaDot);
    """

    dxdt = xDot;
    ddxdt = Robot.wheel_radius/Robot.I_wheel*(-float(U[0])-Robot.wheel_friction/Robot.wheel_radius*xDot);#*np.cos(phi);
    dydt = yDot;
    ddydt = 0;#wheel_R*thetaDot*np.sin(phi);
    dphidt = phiDot;
    ddphidt = 1/game_field.I_ramp*(-game_field.K_ramp*phi-game_field.D_ramp*phiDot)
    dthdt = thetaDot;
    ddthdt = 1/Robot.I_wheel*(float(U[0]) -Robot.wheel_friction*thetaDot)
    dth_armdt = arm_thetaDot;
    ddth_armdt = 1/Robot.I_arm * (float(U[2]) -Robot.arm_friction*arm_thetaDot)
    dl1dt = l1Dot;
    ddl1dt = 1/(Robot.claw_mass+Robot.Extended_arm_mass) * (float(U[3])-Robot.slide_friction*l1Dot);

    if((l1>=Robot.arm_length_max and l1Dot>0) or (l1<=Robot.arm_length_min and l1Dot<0)):
        dl1dt = 0

    DXDT = [dxdt,ddxdt,dydt,ddydt,dphidt,ddphidt,dthdt,ddthdt,0,0,0,0,dth_armdt,ddth_armdt,dl1dt,ddl1dt];

    # 	pdb.set_trace()
    return DXDT



""" How many imputs does the robot have
U[0]: Left Drivebase motors
U[1]: Right Drivebase motors
U[2]: Arm motor 1 (base pivot)
U[3]: Arm piston  (arm extension)
U[4]: Arm motor 2 (wrist joint)
U[5]: roller claw motor 1 (widen base)
U[6]: roller claw motor 2 (roller wheels)
"""
def Inputs(x,xDot,y,yDot,theta,thetaDot,arm_theta,arm_thetaDot,l1,l1Dot):
    global scored;
    if(not scored):
        # the current priority is to score, so implement a controller for scoring
        U = scoring_Controller(x,xDot,y,yDot,theta,thetaDot,arm_theta,arm_thetaDot,l1,l1Dot);
    elif(not mobility):
        # the current priority is to get mobility point, so implement a controller to leave the grid
        U = mobility_Controller(x,xDot,y,yDot,theta,thetaDot,arm_theta,arm_thetaDot,l1,l1Dot);
    # 		pdb.set_trace();
    else:
        # the current priority is to balance, so implement a controller for balancing
        U = balanced_Controller(x,xDot,y,yDot,theta,thetaDot,arm_theta,arm_thetaDot,l1,l1Dot);
    # 		pdb.set_trace();
    # 	pdb.set_trace();
    if(U[2]>= Robot.maxArmTorque):
         U[2]= Robot.maxArmTorque;
    return U;

def scoring_Controller(x,xDot,y,yDot,theta,thetaDot,arm_theta,arm_thetaDot,l1,l1Dot):
    global scored;
    # To score we must drive forward, position the arm, and release the cube on the top level
    x_ref = 0; # this is the target position to score (DRIVE ROBOT TO THIS POINT)
    arm_theta_ref = 0; # this is the target arm orientation (DRIVE THE ARM TO THIS ORIENTATION)
    arm_length_ref = Robot.arm_length_min; # this is the target arm length

    K_drive = -0.3; # this is the control gain for driving the robot during scroring
    K_speed = 0.4; # this is the control gain for the robots speed.
    K_arm_orient = 1.5; # this is the control gain for the robot arm orientation
    K_arm_ang_vel = 0.5; # this is the control gain for the robot's arm angulat velocity
    U = np.zeros((6,1)); # Predefine the robot's control input as a vector

    # compute input
    ref = np.array([x_ref,0,arm_theta_ref,0,arm_length_ref,0]); # vector of references
    X = np.array([x,xDot,arm_theta,arm_thetaDot,l1,l1Dot]); # vector of actual states
    err = ref-X;
    #need to make sure to set max arm length after arm is at certain angle
    #if arm theta ref within a certain range?
    

    U[0] = K_drive*err[0] + K_speed*err[1];
    U[1] = -U[0];
    U[2] = K_arm_orient*err[2] + K_arm_ang_vel*err[3];

    if(arm_theta <= 5*(math.pi)/180):
       ref[4]=Robot.arm_length_max
        #pdb.set_trace()
    if(ref[4]==Robot.arm_length_max):
        U[3] = Robot.pistonForce;
    elif(ref[4]==Robot.arm_length_min):
        U[3] = -Robot.pistonForce;


    # Check if robot has scored (ARE THE STATES EQUAL TO THE REFERENCE VALUES)
    if(np.abs(err[0])<=0.065 and np.abs(err[1])<=0.15):
        scored = True;
    # 		pdb.set_trace();
    # 	pdb.set_trace();
    return U

def mobility_Controller(x,xDot,y,yDot,theta,thetaDot,arm_theta,arm_thetaDot,l1,l1Dot):
    global mobility
    # To get mobility points we must drive backwards over the charging station and leave the community
    x_ref = (96.75+24)*(0.0254); # this is the target position to score (DRIVE ROBOT TO THIS POINT)
    arm_theta_ref = 70*(math.pi/180); # this is the target arm orientation (DRIVE THE ARM TO THIS ORIENTATION)
    arm_length_ref = Robot.arm_length_min; # this is the target arm length

    K_drive = -1.5; # this is the control gain for driving the robot during scroring
    K_speed = 0.6; # this is the control gain for the robots speed.
    K_arm_orient = 1; # this is the control gain for the robot arm orientation
    K_arm_ang_vel = 0.5; # this is the control gain for the robot's arm angulat velocity

    U = np.zeros((6,1)); # Predefine the robot's control input as a vector
    # compute input
    ref = np.array([x_ref,0,arm_theta_ref,0,arm_length_ref,0]); # vector of references
    X = np.array([x,xDot,arm_theta,arm_thetaDot,l1,l1Dot]); # vector of actual states
    err = ref-X;

    U[0] = K_drive*err[0] + K_speed*err[1];
    U[1] = -U[0];
    U[2] = K_arm_orient*err[2] + K_arm_ang_vel*err[3];

    # 	pdb.set_trace()
    if(ref[4]==Robot.arm_length_max):
        U[3] = Robot.pistonForce;
    elif(ref[4]==Robot.arm_length_min):
        U[3] = -Robot.pistonForce;


    if(np.abs(err[0])<=0.098*x_ref and np.abs(err[1])<=0.005):
        mobility = True;

    return U


def balanced_Controller(x,xDot,y,yDot,theta,thetaDot,balanced,arm_theta,arm_thetaDot,l1,l1Dot):
    # To get mobility points we must drive backwards over the charging station and leave the community
    x_ref = (56.75)*(0.0254); # this is the target position to score (DRIVE ROBOT TO THIS POINT)
    arm_theta_ref = 70*(math.pi/180); # this is the target arm orientation (DRIVE THE ARM TO THIS ORIENTATION)
    arm_length_ref = Robot.arm_length_min; # this is the target arm length

    K_drive = -1.5; # this is the control gain for driving the robot during scroring
    K_speed = 0.6; # this is the control gain for the robots speed.
    U = np.zeros((6,1)); # Predefine the robot's control input as a vector
    # compute input
    ref = np.array([x_ref,0,arm_theta_ref,0,arm_length_ref,0]); # vector of references
    X = np.array([x,xDot,arm_theta,arm_thetaDot,l1,l1Dot]); # vector of actual states
    err = ref-X;

    U[0] = K_drive*err[0] + K_speed*err[1];
    U[1] = -U[0];
  

    return U

scored,mobility,balanced = False,False,False;
main()

