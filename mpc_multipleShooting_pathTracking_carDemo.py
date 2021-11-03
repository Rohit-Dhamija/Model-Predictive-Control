#!/usr/bin/env python
import rospy
from prius_msgs.msg import Control    
from nav_msgs.msg import Odometry 
from nav_msgs.msg import Path
import casadi as ca
import numpy as np    
import math   
from scipy.spatial import KDTree
from tf.transformations import euler_from_quaternion 

pi = math.pi
inf = np.inf

"""# variable parameters 
"""
n_states = 4                                                                    
n_controls = 2


N =73                                                                           # Prediction horizon(same as control horizon)
error_allowed = 0.1
U_ref = np.array([0,0], dtype ='f')                                             # U_ref contains referance acc and steer
V_ref = 10                                                                      # referance velocity 


Q_x = 3000                                                                      # gains to control error in x,y,V,theta during motion
Q_y = 3000 
Q_V = 1000000                                                                          
Q_theta = 200 

R1 = 100000                                                                     # gains to control acc and steer                                                                                                           
R2 = 10000

error_allowed_in_g = 1e-100                                                     # error in contraints





"""# parameters that depend on simulator 
"""

throttle_constant = 4.2                                                      # throttle_constant = acc/throttle
steer_constant = 40*pi/180                                                   # steer_constant = steer/steer_input
l_car = 3

n_bound_var = n_states              
x_bound_max = inf                                                            # enter x and y bounds when limited world like indoor environments                     
x_bound_min = -inf                     
y_bound_max = inf                      
y_bound_min = -inf 
V_bound_max = inf                                                                           
V_bound_min = -inf                     
theta_bound_max = inf                     
theta_bound_min = -inf  

acc_max = 4.2                                                                # throttle_max = 1                                                                                                                                             
acc_min = -acc_max                                                                                                                                                                                                  
steer_max = 40*pi/180                                                        # steer_input_max = 1                                                   
steer_min = -steer_max



global x,y,V,theta,throttle,steer_input                                      # (x,y,V,theta) will store the current position,current speed and orientation 
                                                                             # throttle and steer_input will store the inputs to the vehicle 
                                                                             
global total_path_points                                                                                                                                    
total_path_points = 0                                                                                                                                                                            
global path                                                                                                                                       

def pathfunc(Path):              
                                                                                                                         
    global total_path_points,path
    if total_path_points == 0:        
        total_path_points = len(Path.poses)
        path = np.zeros((total_path_points,2))													

    for i in range(0,total_path_points):                                                                                                                
        path[i][0] = Path.poses[i].pose.position.x
        path[i][1] = Path.poses[i].pose.position.y   


def odomfunc(odom):
    
    global x,y,V,theta
    x = odom.pose.pose.position.x 
    y = odom.pose.pose.position.y 
    V = math.sqrt(odom.twist.twist.linear.x**2 + odom.twist.twist.linear.y**2)

    quaternions =  odom.pose.pose.orientation                                                       
    quaternions_list = [quaternions.x,quaternions.y,quaternions.z,quaternions.w]
    roll,pitch,yaw = euler_from_quaternion(quaternions_list)
    theta = yaw
	

def my_mainfunc():
    rospy.init_node('mpc_multipleShooting_pathTracking_carDemo', anonymous=True)
    rospy.Subscriber('/base_pose_ground_truth' , Odometry, odomfunc)            
    rospy.Subscriber('/astroid_path', Path, pathfunc)

    instance = rospy.Publisher('prius', Control, queue_size=10)

    rate = rospy.Rate(10)
    rate.sleep()                                                                                 #rate.sleep() to run odomfunc once 

    msg = Control()

    path_resolution =  ca.norm_2(path[0,0:2] - path[1,0:2])                                                                                        
    global delta_T                                                                               #timestamp bw two predictions                                                                                                                  
    delta_T = ( path_resolution / ((V_ref)) )/10                                                                                                                                                                                                           
    
    

    """MPC"""

    x_casadi =ca.SX.sym('x')
    y_casadi = ca.SX.sym('y')
    V_casadi = ca.SX.sym('V')
    theta_casadi = ca.SX.sym('theta')
    states =np.array([(x_casadi),(y_casadi),(V_casadi),(theta_casadi)])
    n_states = states.size   

    acc_casadi =ca.SX.sym('acc')
    steer_casadi = ca.SX.sym('steer')
    controls = np.array([acc_casadi,steer_casadi])
    n_controls = controls.size  

    rhs = np.array([V_casadi*ca.cos(theta_casadi),V_casadi*ca.sin(theta_casadi),acc_casadi,ca.tan(steer_casadi)/l_car])                                              
    f = ca.Function('f',[states,controls],[rhs])                                                                            # function to predict rhs using states and controls        

    U = ca.SX.sym('U', n_controls,N)                                                                                        # For storing predicted controls  
    X =ca.SX.sym('X', n_states, N+1)                                                                                        # For storing predicted states 
    P = ca.SX.sym('P',1, n_states + n_states*(N) + n_controls*(N) )                                                         # For storing odometry, next N path points and next N referance controls                                                  

    
    
    obj = 0
    g = []

    Q = ca.diagcat(Q_x, Q_y,Q_V, Q_theta)   
    R = ca.diagcat(R1, R2) 

    for i in range(0,N):                                                                                                                                                                                                                         
        cost_pred_st = ca.mtimes(  ca.mtimes( (X[0:n_states,i] - P[n_states*(i+1) :n_states*(i+1) + n_states ].reshape((n_states,1)) ).T , Q )  ,  (X[0:n_states,i] - P[n_states*(i+1) :n_states*(i+1) + n_states ].reshape((n_states,1)) )  )  + ca.mtimes(  ca.mtimes( ( (U[0:n_controls,i]) - P[n_states*(N+1)+n_controls*(i):n_states*(N+1)+n_controls*(i) + n_controls].reshape((n_controls,1)) ).T , R )  ,  U[0:n_controls,i] - P[n_states*(N+1)+n_controls*(i):n_states*(N+1)+n_controls*(i) + n_controls].reshape((n_controls,1))  )  
        obj = obj + cost_pred_st  

    
  
    pred_st = np.zeros((n_states,1))     
    for i in range(0,N+1):                                                                                                   # adding contraints so the predictions are in sync with vehicle model  
        if i == 0:
    	    g = ca.vertcat( g,( X[0:n_states,i] - P[0:n_states].reshape((n_states,1)) )  )                                                             
        else:
            # f_value = f(X[0:n_states,i-1],U[0:n_controls,i-1])                                                             # euler method not used 
            # pred_st = X[0:n_states,i-1] + delta_T*f_value                                                                    

            K1 = f(X[0:n_states,i-1],U[0:n_controls,i-1])                                                                    # Runge Kutta method of order 4 
            K2 = f(X[0:n_states,i-1] + np.multiply(K1,delta_T/2),U[0:n_controls,i-1])                                              
            K3 = f(X[0:n_states,i-1] + np.multiply(K2,delta_T/2),U[0:n_controls,i-1])                                           
            K4 = f(X[0:n_states,i-1] + np.multiply(K3,delta_T),U[0:n_controls,i-1])                                                  
            pred_st = X[0:n_states,i-1] + (delta_T/6)*(K1+2*K2+2*K3+K4)                                                      # predicted state                                              
             
            g = ca.vertcat( g,(X[0:n_states,i] - pred_st[0:n_states].reshape((n_states,1)) )  )                      
 
    OPT_variables = X.reshape((n_states*(N+1),1))          
    OPT_variables = ca.vertcat( OPT_variables, U.reshape((n_controls*N,1)) )     

    nlp_prob ={
            'f':obj,
            'x':OPT_variables,
            'g':g,
            'p':P
            }
    
    opts = {
             'ipopt':
            {
              'max_iter': 100,
              'print_level': 0,
              'acceptable_tol': 1e-8,
              'acceptable_obj_change_tol': 1e-6
            },
             'print_time': 0
           }
    
    
    solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)
    
    
    
    lbg = ca.DM.zeros(((n_states)*(N+1),1))                                                                           # bounds on g                                                                                                           
    ubg = ca.DM.zeros(((n_states)*(N+1),1))                              

    lbg[0:(n_states)*(N+1)] = - error_allowed_in_g                                                                                                  
    ubg[0:(n_states)*(N+1)] =  error_allowed_in_g                                                                                                    
    
    lbx = ca.DM.zeros((n_states*(N+1) + n_controls*N,1))                                                              # bounds on X  
    ubx = ca.DM.zeros((n_states*(N+1) + n_controls*N,1)) 

    lbx[0:n_bound_var*(N+1):n_states] = x_bound_min                     
    ubx[0:n_bound_var*(N+1):n_states] = x_bound_max                     
    lbx[1:n_bound_var*(N+1):n_states] = y_bound_min                     
    ubx[1:n_bound_var*(N+1):n_states] = y_bound_max                     
    lbx[2:n_bound_var*(N+1):n_states] = V_bound_min                 
    ubx[2:n_bound_var*(N+1):n_states] = V_bound_max 
    lbx[3:n_bound_var*(N+1):n_states] = theta_bound_min                 
    ubx[3:n_bound_var*(N+1):n_states] = theta_bound_max                

    lbx[n_bound_var*(N+1):(n_bound_var*(N+1)+n_controls*N):n_controls] = acc_min                       
    ubx[(n_bound_var*(N+1)):(n_bound_var*(N+1)+n_controls*N):n_controls] = acc_max                     
    lbx[(n_bound_var*(N+1)+1):(n_bound_var*(N+1)+n_controls*N):n_controls] = steer_min               
    ubx[(n_bound_var*(N+1)+1):(n_bound_var*(N+1)+n_controls*N):n_controls] = steer_max  


    X_init = np.array([x,y,V,theta], dtype = 'f')                                                                                                               
    X_target = np.array([ path[total_path_points-1][0], path[total_path_points-1][1],0, 0 ]  , dtype = 'f')            #theta_target not considered for stopping condition and Velocity at target is zero !  

    P = X_init      
    close_index = KDTree(path).query(P[0:n_states-2])[1]

    for i in range(0,N):                                                                              
        P = ca.vertcat(P,path[close_index+i,0:2])        
        P = ca.vertcat(P,V_ref)                                                                                                                                                                
        P = ca.vertcat(P, math.atan((path[close_index+i+1][1] - path[close_index+i][1])/(path[close_index+i+1][0] - path[close_index+i][0])) )    
        
    for i in range(0,N):                                                                              
        P = ca.vertcat(P, U_ref[0])                                                                   
        P = ca.vertcat(P, U_ref[1])    

    initial_X = ca.DM.zeros((n_states*(N+1)))                                                                           #all initial search values of predicted states are X_init
    initial_X[0:n_states*(N+1):n_states] = X_init[0]
    initial_X[1:n_states*(N+1):n_states] = X_init[1]
    initial_X[2:n_states*(N+1):n_states] = X_init[2]
    initial_X[3:n_states*(N+1):n_states] = X_init[3]      

    initial_con = ca.DM.zeros((n_controls*N,1))                                                                         #initial search values of control matrix are zero

    while ( ca.norm_2( P[0:n_states-1].reshape((n_states-1,1)) - X_target[0:n_states-1] ) > error_allowed  ) :                                                                                         
           
        args = {
                'lbx':lbx,
                'lbg':lbg,	    
                'ubx':ubx,
                'ubg':ubg,
                'p':P,
                'x0':ca.vertcat(initial_X,initial_con),                                      
               }
        
        sol = solver(
                        
                     x0=args['x0'],
                       
                     lbx=args['lbx'],
                     ubx=args['ubx'],
                    
                     lbg=args['lbg'],
                     ubg=args['ubg'],
                     p=args['p']
                          
                    )       
      
        X_U_sol = sol['x']

        acc = (X_U_sol[n_states*(N+1)].full())[0][0]      
        steer = (X_U_sol[n_states*(N+1)+1].full())[0][0]

        throttle = acc / throttle_constant
        steer_input = steer/steer_constant

        msg.throttle = throttle                                  
        msg.brake = 0.0 
        msg.steer = steer_input
        msg.shift_gears =2
        if throttle < 0:
            msg.shift_gears =3                                              # reverse gear
            throttle = -throttle
        
        instance.publish(msg)
        print ('Velocity (in m/s)  = ',round(V,2))
        

        P[0:n_states] = [x,y,V,theta] 

        close_index = KDTree(path).query(np.array([x,y]))[1]

        if N+(close_index) < total_path_points :                                                                                # Updating P for next N path points and next N reference controls
            P[n_states:n_states*(N+1):n_states] = path[close_index:N+close_index,0] 
            P[n_states+1:n_states*(N+1):n_states] = path[close_index:N+close_index,1]
            for i in range(0,N):                                                                
                P[n_states*(i+1+1)-2] = V_ref                                                                                                                          
                P[n_states*(i+1+1)-1] = math.atan( (path[i+close_index+1][1] - path[i+close_index][1])/(path[i+close_index+1][0] - path[i+close_index][0] + 1e-9) )  

            P[n_states*(N+1):n_states*(N+1)+n_controls*(N-1)]= P[n_states*(N+1)+n_controls:n_states*(N+1)+n_controls*(N)]                                                                                                                                                                                                             
            P[n_states*(N+1)+n_controls*(N-1):n_states*(N+1)+n_controls*(N)] = U_ref   

        else:
            print ("The end point in inside horizon, slowing down")
            P[n_states:n_states*(N)] = P[n_states*2:n_states*(N+1)]                                                                  
            P[n_states*(N):n_states*(N+1)-2] = path[(total_path_points-1),0:2]                                                                                                                                                                                                                                                                                                           
            P[n_states*(N+1)-2] = 0 #V_ref                                                                    #we need to stop the bot at end, hence referance velocity 0 at end                                                                                                                              #
            P[n_states*(N+1)-1] = math.atan( (path[total_path_points-1][1] - path[total_path_points-1-1][1])/(path[total_path_points-1][0] - path[total_path_points-1-1][0]) )
                
            P[n_states*(N+1):n_states*(N+1)+n_controls*(N-1)]= P[n_states*(N+1)+n_controls:n_states*(N+1)+n_controls*(N)]                                                                                                                                                                                                                  
            P[n_states*(N+1)+n_controls*(N-1):n_states*(N+1)+n_controls*(N)] = U_ref       
              
        for i in range(0,N*n_states):                              
            initial_X[i] = X_U_sol[i+n_states]                                                                #initial search value of state for next iteration should be the predicted one for that iteration         

        for i in range(0,(N-1)*n_controls):                      
            initial_con[i] = X_U_sol[n_states*(N+1)+i+n_controls]                                             #initial search value of control for next iteration should be the predicted one for that iteration

        rate.sleep()

    print ("PATH TRACKED")
    msg.throttle = 0                                                                    # stopping the vehicle                       
    msg.brake = 1 
    msg.steer = 0
    msg.shift_gears =1                                                                  # nuetral gear    
    instance.publish(msg)

      


if __name__ == '__main__':
   
    try:
        my_mainfunc()
    except rospy.ROSInterruptException:
        pass
