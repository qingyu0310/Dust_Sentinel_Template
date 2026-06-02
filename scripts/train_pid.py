import numpy as np
import keras

MAX_VELOCITY = 0.5
MAX_CURRENT = 20.0
MAX_TORQUE = 6.0
MAX_OMEGA = 50.0   # 先按你的实测范围改

def nn_data(target_v, now_v, now_torque, target_current, now_omega):
    target_v = target_v / MAX_VELOCITY
    now_v = now_v / MAX_VELOCITY
    v_err = target_v - now_v

    now_torque = now_torque / MAX_TORQUE
    now_omega = now_omega / MAX_OMEGA

    x = np.stack([
        target_v, now_v, v_err, now_torque, now_omega
    ], axis=1)

    target_current = target_current / MAX_CURRENT
    y = np.reshape(target_current, (-1, 1))

    return x, y

def build_model(input_dim):
    model = keras.Sequential([
        keras.layers.Input(shape=(input_dim,)),
        keras.layers.Dense(16, activation='tanh'),
        keras.layers.Dense(16, activation='tanh'),
        keras.layers.Dense(1, activation='tanh'),
    ])
    return model




























