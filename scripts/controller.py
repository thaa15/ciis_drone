#!/usr/bin/env python3
import rospy
from ciis_drone.msg import Motor
import pandas as pd
import numpy as np
from scipy.stats import zscore
from sklearn.preprocessing import MinMaxScaler
import tensorflow as tf
import os
import serial
import time

np.seterr(divide='ignore', invalid='ignore')
maxz = np.zeros(4);minz = np.zeros(4);count=1
IV1 = [];IV2 = [];IV3 = [];IV4 = [];IV5 = []
physical_devices = tf.config.list_physical_devices('GPU')
tf.config.experimental.set_memory_growth(physical_devices[0], enable=True)
esp32 = serial.Serial('/dev/ttyUSB0', 115200, timeout=1000)

def pwm_sender():
    global count,maxz,minz
    rospy.init_node('model_node', anonymous=True)
    pub = rospy.Publisher('pwm', Motor, queue_size=10)
    mtr = Motor()
    data_pwm = pd.read_csv("/home/ciis-ftui/skripsi2023_ws/src/ciis_drone/Data Terbang/data/04_40_31_actuator_outputs_0.csv")
    data_attitude_input = pd.read_csv("/home/ciis-ftui/skripsi2023_ws/src/ciis_drone/Data Terbang/data/04_40_31_vehicle_rates_setpoint_0.csv")
    data_position_input = pd.read_csv("/home/ciis-ftui/skripsi2023_ws/src/ciis_drone/Data Terbang/data/04_40_31_vehicle_local_position_0.csv")

    ## DATA PWM 4 MOTOR
    pwm_0 = np.array(data_pwm['output[0]'])
    pwm_1 = np.array(data_pwm['output[1]'])
    pwm_2 = np.array(data_pwm['output[2]'])
    pwm_3 = np.array(data_pwm['output[3]'])
    pos_x = np.array(data_position_input['x'])
    pos_y = np.array(data_position_input['y'])
    pos_z = np.array(data_position_input['z'])
    pos_x[:len(data_pwm)]
    pos_y[:len(data_pwm)]
    pos_z[:len(data_pwm)]
    speed = np.sqrt(np.square(pos_x)+np.square(pos_y)+np.square(pos_z))

    zscore(pwm_0);zscore(pwm_1);zscore(pwm_2);zscore(pwm_3);zscore(speed)

    scaler = MinMaxScaler(feature_range=(-1,1))
    maxz = [np.max(pwm_0),np.max(pwm_1),np.max(pwm_2),np.max(pwm_3)]
    minz = [np.min(pwm_0),np.min(pwm_1),np.min(pwm_2),np.min(pwm_3)]
    pwm_0 = scaler.fit_transform(pwm_0.reshape(-1,1))
    pwm_1 = scaler.fit_transform(pwm_1.reshape(-1,1))
    pwm_2 = scaler.fit_transform(pwm_2.reshape(-1,1))
    pwm_3 = scaler.fit_transform(pwm_3.reshape(-1,1))
    speed = scaler.fit_transform(speed.reshape(-1,1))

    ## DATA ROLL, PITCH, YAW
    avg_data = int(len(data_attitude_input)/len(data_pwm))
    roll_input_data = data_attitude_input['roll']
    pitch_input_data = data_attitude_input['pitch']
    yaw_input_data = data_attitude_input['yaw']

    data_roll = 0;data_roll_arr = []
    data_pitch = 0;data_pitch_arr = []
    data_yaw = 0;data_yaw_arr=[]

    ## ROLL DATA
    for i,v in roll_input_data.items():
        if i % 5 != 0:
            data_roll += v
        else:
            data_roll = data_roll/avg_data
            data_roll_arr.append(data_roll)
            data_roll = 0

    ## YAW DATA
    for i,v in yaw_input_data.items():
        if i % 5 != 0:
            data_yaw += v
        else:
            data_yaw = data_yaw/avg_data
            data_yaw_arr.append(data_yaw)
            data_yaw = 0

    ## PITCH DATA
    for i,v in pitch_input_data.items():
        if i % 5 != 0:
            data_pitch += v
        else:
            data_pitch = data_pitch/avg_data
            data_pitch_arr.append(data_pitch)
            data_pitch = 0

    data_roll_arr = np.array(data_roll_arr)
    data_pitch_arr = np.array(data_pitch_arr)
    data_yaw_arr = np.array(data_yaw_arr)
    zscore(data_roll_arr);zscore(data_pitch_arr);zscore(data_yaw_arr)

    roll = scaler.fit_transform(data_roll_arr.reshape(-1,1))
    pitch = scaler.fit_transform(data_pitch_arr.reshape(-1,1))
    yawCos = np.cos(np.rad2deg(data_yaw_arr))
    yawSin = np.sin(np.rad2deg(data_yaw_arr))
    pwm0_NN1 = np.zeros(len(pwm_0))
    pwm1_NN1 = np.zeros(len(pwm_1))
    pwm2_NN1 = np.zeros(len(pwm_2))
    pwm3_NN1 = np.zeros(len(pwm_3))
    pwm0_NN2 = np.zeros(len(pwm_0))
    pwm1_NN2 = np.zeros(len(pwm_1))
    pwm2_NN2 = np.zeros(len(pwm_2))
    pwm3_NN2 = np.zeros(len(pwm_3))
    pwm0_NN3 = np.zeros(len(pwm_0))
    pwm1_NN3 = np.zeros(len(pwm_1))
    pwm2_NN3 = np.zeros(len(pwm_2))
    pwm3_NN3 = np.zeros(len(pwm_3))
    pwm0_NN4 = np.zeros(len(pwm_0))
    pwm1_NN4 = np.zeros(len(pwm_1))
    pwm2_NN4 = np.zeros(len(pwm_2))
    pwm3_NN4 = np.zeros(len(pwm_3))
    pwm0_NN5 = np.zeros(len(pwm_0))
    pwm1_NN5 = np.zeros(len(pwm_1))
    pwm2_NN5 = np.zeros(len(pwm_2))
    pwm3_NN5 = np.zeros(len(pwm_3))
    roll_NN1 = np.zeros(len(roll))
    pitch_NN2 = np.zeros(len(pitch))
    yawCos_NN3 = np.zeros(len(yawCos))
    yawSin_NN4 = np.zeros(len(yawSin))
    speed_NN5 = np.zeros(len(speed))
    NN1 = tf.keras.models.load_model('/home/ciis-ftui/skripsi2023_ws/src/ciis_drone/Model/NN_IV_1_new3.h5')
    NN2 = tf.keras.models.load_model('/home/ciis-ftui/skripsi2023_ws/src/ciis_drone/Model/NN_IV_2_new3.h5')
    NN3 = tf.keras.models.load_model('/home/ciis-ftui/skripsi2023_ws/src/ciis_drone/Model/NN_IV_3_new3.h5')
    NN4 = tf.keras.models.load_model('/home/ciis-ftui/skripsi2023_ws/src/ciis_drone/Model/NN_IV_4_new3.h5')
    NN5 = tf.keras.models.load_model('/home/ciis-ftui/skripsi2023_ws/src/ciis_drone/Model/NN_IV_5_new3.h5')
    NN6 = tf.keras.models.load_model('/home/ciis-ftui/skripsi2023_ws/src/ciis_drone/Model/NN_IV_6_new3.h5')
    rate = rospy.Rate(75)
    counter = 0
    esp32.write("YES".encode('utf-8'))
    while not rospy.is_shutdown():
        # rospy.loginfo(database[0])
        if count != 570:
            if count > 2:
                x_IV1 = np.zeros((3,5))
                x_IV2 = np.zeros((3,5))
                x_IV3 = np.zeros((3,5))
                x_IV4 = np.zeros((3,5))
                x_IV5 = np.zeros((3,5))

                x_IV1[:,0] = roll[counter:counter+3].flatten()
                x_IV1[:,1] = pwm0_NN1[counter:counter+3].flatten()
                x_IV1[:,2] = pwm1_NN1[counter:counter+3].flatten()
                x_IV1[:,3] = pwm2_NN1[counter:counter+3].flatten()
                x_IV1[:,4] = pwm3_NN1[counter:counter+3].flatten()

                x_IV2[:,0] = pitch[counter:counter+3].flatten()
                x_IV2[:,1] = pwm0_NN2[counter:counter+3].flatten()
                x_IV2[:,2] = pwm1_NN2[counter:counter+3].flatten()
                x_IV2[:,3] = pwm2_NN2[counter:counter+3].flatten()
                x_IV2[:,4] = pwm3_NN2[counter:counter+3].flatten()

                x_IV3[:,0] = yawCos[counter:counter+3].flatten()
                x_IV3[:,1] = pwm0_NN3[counter:counter+3].flatten()
                x_IV3[:,2] = pwm1_NN3[counter:counter+3].flatten()
                x_IV3[:,3] = pwm2_NN3[counter:counter+3].flatten()
                x_IV3[:,4] = pwm3_NN3[counter:counter+3].flatten()

                x_IV4[:,0] = yawSin[counter:counter+3].flatten()
                x_IV4[:,1] = pwm0_NN4[counter:counter+3].flatten()
                x_IV4[:,2] = pwm1_NN4[counter:counter+3].flatten()
                x_IV4[:,3] = pwm2_NN4[counter:counter+3].flatten()
                x_IV4[:,4] = pwm3_NN4[counter:counter+3].flatten()

                x_IV5[:,0] = speed[counter:counter+3].flatten()
                x_IV5[:,1] = pwm0_NN5[counter:counter+3].flatten()
                x_IV5[:,2] = pwm1_NN5[counter:counter+3].flatten()
                x_IV5[:,3] = pwm2_NN5[counter:counter+3].flatten()
                x_IV5[:,4] = pwm3_NN5[counter:counter+3].flatten()
                
                x_IV1 = [x_IV1]
                x_IV2 = [x_IV2]
                x_IV3 = [x_IV3]
                x_IV4 = [x_IV4]
                x_IV5 = [x_IV5]

                x_IV1 = np.asarray(x_IV1,dtype='float64')
                x_IV2 = np.asarray(x_IV2,dtype='float64')
                x_IV3 = np.asarray(x_IV3,dtype='float64')
                x_IV4 = np.asarray(x_IV4,dtype='float64')
                x_IV5 = np.asarray(x_IV5,dtype='float64')

                pred1 = NN1.predict(x=[x_IV1],verbose=0)
                pred2 = NN2.predict(x=[x_IV2],verbose=0)
                pred3 = NN3.predict(x=[x_IV3],verbose=0)
                pred4 = NN4.predict(x=[x_IV4],verbose=0)
                pred5 = NN5.predict(x=[x_IV5],verbose=0)

                pwm0_NN1[counter+3] = pred1[:,0][0]
                pwm1_NN1[counter+3] = pred1[:,1][0]
                pwm2_NN1[counter+3] = pred1[:,2][0]
                pwm3_NN1[counter+3] = pred1[:,3][0]

                pwm0_NN2[counter+3] = pred2[:,0][0]
                pwm1_NN2[counter+3] = pred2[:,1][0]
                pwm2_NN2[counter+3] = pred2[:,2][0]
                pwm3_NN2[counter+3] = pred2[:,3][0]

                pwm0_NN3[counter+3] = pred3[:,0][0]
                pwm1_NN3[counter+3] = pred3[:,1][0]
                pwm2_NN3[counter+3] = pred3[:,2][0]
                pwm3_NN3[counter+3] = pred3[:,3][0]

                pwm0_NN4[counter+3] = pred4[:,0][0]
                pwm1_NN4[counter+3] = pred4[:,1][0]
                pwm2_NN4[counter+3] = pred4[:,2][0]
                pwm3_NN4[counter+3] = pred4[:,3][0]

                pwm0_NN5[counter+3] = pred5[:,0][0]
                pwm1_NN5[counter+3] = pred5[:,1][0]
                pwm2_NN5[counter+3] = pred5[:,2][0]
                pwm3_NN5[counter+3] = pred5[:,3][0]

                x_NN = np.zeros((len(pwm_0),20))
                x_NN[:,0] = pwm0_NN1[counter]
                x_NN[:,1] = pwm1_NN1[counter]
                x_NN[:,2] = pwm2_NN1[counter]
                x_NN[:,3] = pwm3_NN1[counter]
                x_NN[:,4] = pwm0_NN2[counter]
                x_NN[:,5] = pwm1_NN2[counter]
                x_NN[:,6] = pwm2_NN2[counter]
                x_NN[:,7] = pwm3_NN2[counter]
                x_NN[:,8] = pwm0_NN3[counter]
                x_NN[:,9] = pwm1_NN3[counter]
                x_NN[:,10] = pwm2_NN3[counter]
                x_NN[:,11] = pwm3_NN3[counter]
                x_NN[:,12] = pwm0_NN4[counter]
                x_NN[:,13] = pwm1_NN4[counter]
                x_NN[:,14] = pwm2_NN4[counter]
                x_NN[:,15] = pwm3_NN4[counter]
                x_NN[:,16] = pwm0_NN5[counter]
                x_NN[:,17] = pwm1_NN5[counter]
                x_NN[:,18] = pwm2_NN5[counter]
                x_NN[:,19] = pwm3_NN5[counter]
                pred_NN = NN6.predict(x = [x_NN],verbose=0)
                hasil_pwm1 = (pred_NN[:,0][0] + 1)*(maxz[0] - minz[0])/2 + minz[0]
                hasil_pwm2 = (pred_NN[:,1][0] + 1)*(maxz[1] - minz[1])/2 + minz[1]
                hasil_pwm3 = (pred_NN[:,2][0] + 1)*(maxz[2] - minz[2])/2 + minz[2]
                hasil_pwm4 = (pred_NN[:,3][0] + 1)*(maxz[3] - minz[3])/2 + minz[3]
                mtr.pwm1 = hasil_pwm1
                mtr.pwm2 = hasil_pwm2
                mtr.pwm3 = hasil_pwm3
                mtr.pwm4 = hasil_pwm4
                toSend = f"<{str(int(hasil_pwm1))},{str(int(hasil_pwm2))},{str(int(hasil_pwm3))},{str(int(hasil_pwm4))}>"
                esp32.write(toSend.encode('utf-8'))
                counter = counter + 1
            # rospy.loginfo(mtr)
            pub.publish(mtr)
            count = count + 1
        else:
            toSend = f"<END>"
            esp32.write(toSend.encode('utf-8'))
            mtr.flight = False
        # pub.publish(mtr)
        rate.sleep()

if __name__ == '__main__':
    try:
        pwm_sender()
    except rospy.ROSInterruptException:
        rospy.loginfo("Inverse NN Failed to Start")
        pass
