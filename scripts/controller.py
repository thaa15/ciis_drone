#!/usr/bin/env python3
import rospy
from ciis_drone.msg import Motor
import pandas as pd
import numpy as np
from scipy.stats import zscore
from sklearn.preprocessing import MinMaxScaler
import tensorflow as tf

max = np.zeros(4);min = np.zeros(4);count=1;T=3
IV1 = [];IV2 = [];IV3 = [];IV4 = [];IV5 = []

def att_command():
    global max, min
    data_pwm = pd.read_csv("./Data Terbang/data/04_40_31_actuator_outputs_0.csv")
    data_attitude_input = pd.read_csv("./Data Terbang/data/04_40_31_vehicle_rates_setpoint_0.csv")
    data_position_input = pd.read_csv("./Data Terbang/data/04_40_31_vehicle_local_position_0.csv")

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
    max = [np.max(pwm_0),np.max(pwm_1),np.max(pwm_2),np.max(pwm_3)]
    min = [np.min(pwm_0),np.min(pwm_1),np.min(pwm_2),np.min(pwm_3)]
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
    return [roll, pitch, yawCos, yawSin, speed,pwm_0,pwm_1,pwm_2,pwm_3]

def model():
    NN1 = tf.keras.models.load_model('Model/NN1.h5')
    NN2 = tf.keras.models.load_model('Model/NN2.h5')
    NN3 = tf.keras.models.load_model('Model/NN3.h5')
    NN4 = tf.keras.models.load_model('Model/NN4.h5')
    NN5 = tf.keras.models.load_model('Model/NN5.h5')
    NN6 = tf.keras.models.load_model('Model/NN6.h5')
    return [NN1,NN2,NN3,NN4,NN5,NN6]

def pwm_sender():
    rospy.init_node('model_node', anonymous=True)
    pub = rospy.Publisher('pwm', Motor, queue_size=10)
    mtr = Motor()
    database = att_command()
    pwm0_NN1 = np.zeros(len(database[5]))
    pwm1_NN1 = np.zeros(len(database[6]))
    pwm2_NN1 = np.zeros(len(database[8]))
    pwm3_NN1 = np.zeros(len(database[9]))
    pwm0_NN2 = np.zeros(len(database[5]))
    pwm1_NN2 = np.zeros(len(database[6]))
    pwm2_NN2 = np.zeros(len(database[8]))
    pwm3_NN2 = np.zeros(len(database[9]))
    pwm0_NN3 = np.zeros(len(database[5]))
    pwm1_NN3 = np.zeros(len(database[6]))
    pwm2_NN3 = np.zeros(len(database[8]))
    pwm3_NN3 = np.zeros(len(database[9]))
    pwm0_NN4 = np.zeros(len(database[5]))
    pwm1_NN4 = np.zeros(len(database[6]))
    pwm2_NN4 = np.zeros(len(database[8]))
    pwm3_NN4 = np.zeros(len(database[9]))
    pwm0_NN5 = np.zeros(len(database[5]))
    pwm1_NN5 = np.zeros(len(database[6]))
    pwm2_NN5 = np.zeros(len(database[8]))
    pwm3_NN5 = np.zeros(len(database[9]))

    pwm0_NN1[:4] = np.append([0],database[5][:3])
    pwm1_NN1[:4] = np.append([0],database[6][:3])
    pwm2_NN1[:4] = np.append([0],database[8][:3])
    pwm3_NN1[:4] = np.append([0],database[9][:3])
    roll_NN1 = database[0].flatten()

    pwm0_NN2[:4] = np.append([0],database[5][:3])
    pwm1_NN2[:4] = np.append([0],database[6][:3])
    pwm2_NN2[:4] = np.append([0],database[8][:3])
    pwm3_NN2[:4] = np.append([0],database[9][:3])
    pitch_NN2 = database[1].flatten()

    pwm0_NN3[:4] = np.append([0],database[5][:3])
    pwm1_NN3[:4] = np.append([0],database[6][:3])
    pwm2_NN3[:4] = np.append([0],database[8][:3])
    pwm3_NN3[:4] = np.append([0],database[9][:3])
    yawCos_NN3 = database[2].flatten()

    pwm0_NN4[:4] = np.append([0],database[5][:3])
    pwm1_NN4[:4] = np.append([0],database[6][:3])
    pwm2_NN4[:4] = np.append([0],database[8][:3])
    pwm3_NN4[:4] = np.append([0],database[9][:3])
    yawSin_NN4 = database[3].flatten()

    pwm0_NN5[:4] = np.append([0],database[5][:3])
    pwm1_NN5[:4] = np.append([0],database[6][:3])
    pwm2_NN5[:4] = np.append([0],database[8][:3])
    pwm3_NN5[:4] = np.append([0],database[9][:3])
    speed_NN5 = database[4].flatten()
    models = model()
    rate = rospy.Rate(75)
    while not rospy.is_shutdown():
        if count != len(database[0]):
            if count == 4:
                pred1 = models[0].predict(x = np.asarray([roll_NN1,pwm0_NN1,pwm1_NN1,pwm2_NN1,pwm3_NN1]),dtype = 'float64')
                pred2 = models[1].predict(x = np.asarray([pitch_NN2,pwm0_NN2,pwm1_NN2,pwm2_NN2,pwm3_NN2]),dtype = 'float64')
                pred3 = models[2].predict(x = np.asarray([yawCos_NN3,pwm0_NN3,pwm1_NN3,pwm2_NN3,pwm3_NN3]),dtype = 'float64')
                pred4 = models[3].predict(x = np.asarray([yawSin_NN4,pwm0_NN4,pwm1_NN4,pwm2_NN4,pwm3_NN4]),dtype = 'float64')
                pred5 = models[4].predict(x = np.asarray([speed_NN5,pwm0_NN5,pwm1_NN5,pwm2_NN5,pwm3_NN5]),dtype = 'float64')
                pwm0_NN1[count] = pred1[:,0];pwm1_NN1[count] = pred1[:,1];pwm2_NN1[count] = pred1[:,2];pwm3_NN1[count] = pred1[:,3]
                pwm0_NN2[count] = pred2[:,0];pwm1_NN2[count] = pred2[:,1];pwm2_NN2[count] = pred2[:,2];pwm3_NN2[count] = pred2[:,3]
                pwm0_NN3[count] = pred3[:,0];pwm1_NN3[count] = pred3[:,1];pwm2_NN3[count] = pred3[:,2];pwm3_NN3[count] = pred3[:,3]
                pwm0_NN4[count] = pred4[:,0];pwm1_NN4[count] = pred4[:,1];pwm2_NN4[count] = pred4[:,2];pwm3_NN4[count] = pred4[:,3]
                pwm0_NN5[count] = pred5[:,0];pwm1_NN5[count] = pred5[:,1];pwm2_NN5[count] = pred5[:,2];pwm3_NN5[count] = pred5[:,3]
                x_train_NN = np.zeros((len(database[5]),20))
                x_train_NN[:,0] = pwm0_NN1
                x_train_NN[:,1] = pwm1_NN1
                x_train_NN[:,2] = pwm2_NN1
                x_train_NN[:,3] = pwm3_NN1
                x_train_NN[:,4] = pwm0_NN2
                x_train_NN[:,5] = pwm1_NN2
                x_train_NN[:,6] = pwm2_NN2
                x_train_NN[:,7] = pwm3_NN2
                x_train_NN[:,8] = pwm0_NN3
                x_train_NN[:,9] = pwm1_NN3
                x_train_NN[:,10] = pwm2_NN3
                x_train_NN[:,11] = pwm3_NN3
                x_train_NN[:,12] = pwm0_NN4
                x_train_NN[:,13] = pwm1_NN4
                x_train_NN[:,14] = pwm2_NN4
                x_train_NN[:,15] = pwm3_NN4
                x_train_NN[:,16] = pwm0_NN5
                x_train_NN[:,17] = pwm1_NN5
                x_train_NN[:,18] = pwm2_NN5
                x_train_NN[:,19] = pwm3_NN5
                pred_NN = models[5].predict(x = [x_train_NN])
                mtr.pwm1 = (pred_NN[count,0] + 1)*(max[0] - min[0])/2 - 1
                mtr.pwm2 = (pred_NN[count,1] + 1)*(max[1] - min[1])/2 - 1
                mtr.pwm3 = (pred_NN[count,2] + 1)*(max[2] - min[2])/2 - 1
                mtr.pwm4 = (pred_NN[count,3] + 1)*(max[3] - min[3])/2 - 1
            # rospy.loginfo(mtr)
            pub.publish(mtr)
            count += 1
        else:
            mtr.flight = False
        pub.publish(mtr)
        rate.sleep()

if __name__ == '__main__':
    try:
        pwm_sender()
    except rospy.ROSInterruptException:
        rospy.loginfo("Inverse NN Failed to Start")
        pass
