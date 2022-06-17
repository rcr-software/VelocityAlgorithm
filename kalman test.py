import math
import matplotlib
import numpy as np
#Kalman variables
#Constants used in Kalman calculations
# float q_k[3][3] = {                                             
#   { 1, 0, 0 },
#   { 0, 0.02, 0 },
#   { 0, 0, 0.2 }
# };

q_k= [                                          
  [ 1, 0, 0 ],
  [ 0, 0.02, 0 ],
  [ 0, 0, 0.2 ]
]

# float r_k[3][3] = {
#   { 0.5, 0, 0 },
#   { 0, 4, 0 },
#   { 0, 0, 7 }
# };

r_k= [                                          
  [ 0.5, 0, 0 ],
  [ 0, 4, 0 ],
  [ 0, 0, 7 ]
]


#
#_  __     _                         ______                _   _
#| |/ /    | |                       |  ____|              | | (_)
#| ' / __ _| |_ __ ___   __ _ _ __   | |__ _   _ _ __   ___| |_ _  ___  _ __  ___
#|  < / _` | | '_ ` _ \ / _` | '_ \  |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
#| . \ (_| | | | | | | | (_| | | | | | |  | |_| | | | | (__| |_| | (_) | | | \__
#|_|\_\__,_|_|_| |_| |_|\__,_|_| |_| |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
#
#************************************************************************
#!
#@brief  Filters the state of the vehicle
#Author: Ben, Denny, and Lydia
#
#************************************************************************
#C++ TO PYTHON CONVERTER NOTE: This was formerly a static local variable declaration (not allowed in Python):
kalman_x_k = [0 for _ in range(3)]
#C++ TO PYTHON CONVERTER NOTE: This was formerly a static local variable declaration (not allowed in Python):
kalman_lastTime = 0
#C++ TO PYTHON CONVERTER NOTE: This was formerly a static local variable declaration (not allowed in Python):
kalman_p_k = [[] for _ in range(3)]

def kalman(encPos, rawState, filteredState):
   
    ### Variables we are needing !!!!
    kalman_x_k = [0 for _ in range(3)]
    
    TIME_DIVISOR=1
    kalman_lastTime=1
       
    
    #    static float x_k[3] = { 0, 0, 0 }
    #    static uint lastTime
    delta_t = None
    z_k = [0 for _ in range(3)]
    #    static float p_k[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } }
    placeHolder_3_1 = [0 for _ in range(3)]
    placeHolder_3_3_1 = [[] for _ in range(3)]
    placeHolder_3_3_2 = [[] for _ in range(3)]
    placeHolder_3_3_3 = [[] for _ in range(3)]
    u_k = None
    c_d = None
    area = None
    q = None
    b_k = [0 for _ in range(3)]
    f_k = [[1, 0, 0], [0, 1, 0], [0, 0, 0]]
    #Convert to numpy matrices
    b_k=np.array(b_k)
    f_k=np.array(f_k)

    #x_k[0] = filteredState->alt
    #x_k[1] = filteredState->vel
    #x_k[2] = filteredState->accel

    z_k[0] = rawState.alt
    z_k[1] = rawState.vel
    z_k[2] = rawState.accel

    delta_t = float((rawState.time - kalman_lastTime)) / TIME_DIVISOR
    kalman_lastTime = rawState.time

    b_k[0] = delta_t *delta_t
    b_k[0] = b_k[0] / 2
    b_k[1] = delta_t
    b_k[2] = 1

    f_k[0][1] = delta_t

##if DEBUG_KALMAN

    print("KALMAN--------------------- \n")
    print("delta_t = ") #temp
    print(delta_t) #temp
    print("t = ") #temp
    print(rawState.time,"\n") #temp
    ##Matrix print
    print(kalman_x_k, 3, 1, "x_k") #temp
    print(float(f_k), 3, 3, "f_k") #temp
    print(b_k, 3, 1, "b_k") #temp
    print(z_k, 3, 1, "z_k") #temp
##endif

    ## Things we are needing
    ####################
    rocket= "???"
    ENC_RANGE=1
    RHO=1
    
    
    #calculate what Kalman thinks the acceleration is
    c_d = rocket.Cd_r *(1 - encPos / ENC_RANGE) + encPos *rocket.Cd_b / ENC_RANGE
    area = rocket.Ar*(1 - encPos / ENC_RANGE) + encPos *rocket.Ab / ENC_RANGE
    q = RHO * rawState.vel * rawState.vel / 2
    u_k = -9.81 - c_d * area * q / rocket.dryMass

    # if acceleration > 10m/s^2 the motor is probably burning and we should add that in to u_k
    if z_k[2] > 10:
        #Serial.println("Burn Phase!"); //errorlog
        u_k += rocket.avgMotorThrust / (rocket.dryMass + rocket.propMass / 2)
    elif (z_k[0] < 20) and (z_k[0] > -20):
        u_k = 0
        #Serial.print("On pad "); //errorlog
        #Serial.println(z_k[0])
    if math.isnan(u_k):
        
##if DEBUG_KALMAN
        print("u_k is nan!","\n")
##endif
        DataLog.logError(NAN_UK)
        u_k = 0

##if DEBUG_KALMAN
    print("\n")
    print("u_k = ")
    print(u_k,"\n")
    print("c_d = ") #temp
    print(c_d,"\n") #temp
    print("area = ") #temp
    print(area,"\n") #temp
    print("q = ") #temp
    print(q,"\n") #temp
##endif

    #Predict----------------------------
    #x_k = u_k*b_k + f_k*x_k
    # Matrix.Scale(float(b_k), 3, 1, u_k)
    b_k = b_k*u_k
    
    # Matrix.Multiply(float(f_k), float(kalman_x_k), 3, 3, 1, float(placeHolder_3_1))
    placeHolder_3_1 = np.dot(f_k , kalman_x_k)
    
    # Matrix.Add(float(b_k), float(placeHolder_3_1), 3, 1, float(kalman_x_k))
    kalman_x_k = b_k+placeHolder_3_1


##if DEBUG_KALMAN
    Matrix.Print(b_k, 3, 1, "u_k*b_k")
    Matrix.Print(placeHolder_3_1, 3, 1, "f_k*x_k")
    Matrix.Print(kalman_x_k, 3, 1, "x_k predict")
##endif

    #p_k = q_k + f_k*p_k*T(f_k)
    Matrix.Multiply(float(f_k), float(kalman_p_k), 3, 3, 3, float(placeHolder_3_3_1))
    Matrix.Transpose(float(f_k), 3, 3, float(placeHolder_3_3_2))
    Matrix.Multiply(float(placeHolder_3_3_1), float(placeHolder_3_3_2), 3, 3, 3, float(placeHolder_3_3_3))
    Matrix.Add(float(placeHolder_3_3_3), float(q_k), 3, 3, float(kalman_p_k))

##if DEBUG_KALMAN
    Matrix.Print(float(kalman_p_k), 3, 3, "p_k predict")
##endif

    #Kalman Gain------------------------
    #p_k*T(h_k) / (r_k + h_k * p_k * T(h_k)) ==
    #p_k / (r_k + p_k)    ..When h_k = eye(3)
    Matrix.Add(float(r_k), float(kalman_p_k), 3, 3, float(placeHolder_3_3_1))
    Matrix.Invert(float(placeHolder_3_3_1), 3)
    Matrix.Multiply(float(kalman_p_k), float(placeHolder_3_3_1), 3, 3, 3, float(k_gain))

##if DEBUG_KALMAN
    Matrix.Print(float(k_gain), 3, 3, "kalman gain")
    #Matrix.Print((float*)placeHolder_3_3_1, 3, 3, "1 / (r_k + p_k)")
##endif

    #Update-----------------------------
    #x_k = k_gain * (z_k - x_k) + x_k
    Matrix.Subtract(float(z_k), float(kalman_x_k), 3, 1, float(placeHolder_3_3_1))
    Matrix.Multiply(float(k_gain), float(placeHolder_3_3_1), 3, 3, 1, float(placeHolder_3_3_2))
    Matrix.Add(float(kalman_x_k), float(placeHolder_3_3_2), 3, 1, kalman_x_k)

    #p_k = p_k - k_gain * p_k
    Matrix.Multiply(float(k_gain), float(kalman_p_k), 3, 3, 1, float(placeHolder_3_3_1))
    Matrix.Subtract(float(kalman_p_k), float(placeHolder_3_3_1), 3, 1, float(kalman_p_k))

    filteredState.alt = kalman_x_k[0]
    filteredState.vel = kalman_x_k[1]
    filteredState.accel = kalman_x_k[2]
    filteredState.time = rawState.time # END kalman()

