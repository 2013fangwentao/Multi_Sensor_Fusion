# -*- coding: utf-8 -*-
import numpy as np
# import matplotlib.pyplot as plt
import sys
import os

TIME_INDEX = 7
POS_INDEX = 8
VEL_INDEX = 11
ATT_INDEX = 14

GYRO_BIAS_INDEX = 17
ACCE_BIAS_INDEX = 20
GYRO_SCALE_INDEX = 23
ACCE_SCALE_INDEX = 26

REF_TIME_INDEX = 0
REF_POS_INDEX = 1
REF_VEL_INDEX = 7
REF_ATT_INDEX = 4


# def plot_imu_error(time, gyro_bias, acce_bias, gyro_scale, acce_scale, is_save,
#                    save_path):
#     plt.figure(11)
#     plt.subplot(3, 1, 1)
#     plt.title("gyro_bias (deg/h)")
#     plt.plot(time, gyro_bias[..., 0])
#     plt.ylabel("x(deg/h)")
#     plt.grid()
#     plt.subplot(3, 1, 2)
#     plt.plot(time, gyro_bias[..., 1])
#     plt.ylabel("y(deg/h)")
#     plt.grid()
#     plt.subplot(3, 1, 3)
#     plt.plot(time, gyro_bias[..., 2])
#     plt.ylabel("z(deg/h)")
#     plt.grid()
#     if (is_save):
#         plt.savefig(os.path.join(save_path, "gyro_bias.png"), format="png")

#     plt.figure(12)
#     plt.subplot(3, 1, 1)
#     plt.title("acce_bias (mGal)")
#     plt.plot(time, acce_bias[..., 0])
#     plt.ylabel("x(mGal)")
#     plt.grid()
#     plt.subplot(3, 1, 2)
#     plt.plot(time, acce_bias[..., 1])
#     plt.ylabel("y(mGal)")
#     plt.grid()
#     plt.subplot(3, 1, 3)
#     plt.plot(time, acce_bias[..., 2])
#     plt.ylabel("z(mGal)")
#     plt.grid()
#     if (is_save):
#         plt.savefig(os.path.join(save_path, "acce_bias.png"), format="png")

#     plt.figure(13)
#     plt.subplot(3, 1, 1)
#     plt.title("gyro_scale (ppm)")
#     plt.plot(time, gyro_scale[..., 0])
#     plt.ylabel("x(ppm)")
#     plt.grid()
#     plt.subplot(3, 1, 2)
#     plt.plot(time, gyro_scale[..., 1])
#     plt.ylabel("y(ppm)")
#     plt.grid()
#     plt.subplot(3, 1, 3)
#     plt.plot(time, gyro_scale[..., 2])
#     plt.ylabel("z(ppm)")
#     plt.grid()
#     if (is_save):
#         plt.savefig(os.path.join(save_path, "gyro_scale.png"), format="png")

#     plt.figure(14)
#     plt.subplot(3, 1, 1)
#     plt.title("acce_scale (ppm)")
#     plt.plot(time, acce_scale[..., 0])
#     plt.ylabel("x(ppm)")
#     plt.grid()
#     plt.subplot(3, 1, 2)
#     plt.plot(time, acce_scale[..., 1])
#     plt.ylabel("y(ppm)")
#     plt.grid()
#     plt.subplot(3, 1, 3)
#     plt.plot(time, acce_scale[..., 2])
#     plt.ylabel("z(ppm)")
#     plt.grid()
#     if (is_save):
#         plt.savefig(os.path.join(save_path, "acce_scale.png"), format="png")
#     plt.show()


# def plot_residual(time, pos, vel, att, is_save, save_path):
#     plt.figure(1)
#     plt.subplot(3, 1, 1)
#     plt.title("pos residual (m)")
#     plt.plot(time, pos[..., 0])
#     plt.ylabel("x(m)")
#     plt.grid()
#     plt.subplot(3, 1, 2)
#     plt.plot(time, pos[..., 1])
#     plt.ylabel("y(m)")
#     plt.grid()
#     plt.subplot(3, 1, 3)
#     plt.plot(time, pos[..., 2])
#     plt.ylabel("z(m)")
#     plt.grid()
#     if (is_save):
#         plt.savefig(os.path.join(save_path, "pos_residual.png"), format="png")

#     plt.figure(2)
#     plt.subplot(3, 1, 1)
#     plt.title("vel residual (m/s)")
#     plt.plot(time, vel[..., 0])
#     plt.ylabel("x(m/s)")
#     plt.grid()
#     plt.subplot(3, 1, 2)
#     plt.plot(time, vel[..., 1])
#     plt.ylabel("y(m/s)")
#     plt.grid()
#     plt.subplot(3, 1, 3)
#     plt.plot(time, vel[..., 2])
#     plt.ylabel("z(m/s)")
#     plt.grid()
#     if (is_save):
#         plt.savefig(os.path.join(save_path, "vel_residual.png"), format="png")

#     plt.figure(3)
#     plt.subplot(3, 1, 1)
#     plt.plot(time, att[..., 0])
#     plt.title("att residual (deg)")
#     plt.ylabel("roll(deg)")
#     plt.grid()
#     plt.subplot(3, 1, 2)
#     plt.plot(time, att[..., 1])
#     plt.ylabel("pitch(deg)")
#     plt.grid()
#     plt.subplot(3, 1, 3)
#     plt.plot(time, att[..., 2])
#     plt.ylabel("heading(deg)")
#     plt.grid()
#     if (is_save):
#         plt.savefig(os.path.join(save_path, "att_residual.png"), format="png")


def compare(result_file,
            truth_file,
            save_path="./",
            start_time=0,
            end_time=86400,
            is_save_picture=False):
    result_data = np.loadtxt(result_file)
    truth_data = np.loadtxt(truth_file)
    data_index = (result_data[..., TIME_INDEX] > start_time)
    data_index_tmp = (result_data[..., TIME_INDEX] < end_time)
    data_index = data_index * data_index_tmp
    refer_index = truth_data[..., REF_TIME_INDEX] > start_time
    refer_index_tmp = truth_data[..., REF_TIME_INDEX] < end_time
    refer_index = refer_index * refer_index_tmp

    data_time = result_data[data_index, TIME_INDEX]
    pos_data = result_data[data_index, POS_INDEX:POS_INDEX + 3]
    vel_data = result_data[data_index, VEL_INDEX:VEL_INDEX + 3]
    att_data = result_data[data_index, ATT_INDEX:ATT_INDEX + 3]
    gyro_bias_data = result_data[data_index, GYRO_BIAS_INDEX:GYRO_BIAS_INDEX +
                                 3]
    gyro_scale_data = result_data[data_index, GYRO_SCALE_INDEX:GYRO_SCALE_INDEX +
                                  3]
    acce_bias_data = result_data[data_index, ACCE_BIAS_INDEX:ACCE_BIAS_INDEX +
                                 3]
    acce_scale_data = result_data[data_index, ACCE_SCALE_INDEX:
                                  ACCE_SCALE_INDEX + 3]

    ref_time = truth_data[refer_index, REF_TIME_INDEX]
    ref_pos_data = truth_data[refer_index, REF_POS_INDEX:REF_POS_INDEX + 3]
    ref_vel_data = truth_data[refer_index, REF_VEL_INDEX:REF_VEL_INDEX + 3]
    ref_att_data = truth_data[refer_index, REF_ATT_INDEX:REF_ATT_INDEX + 3]

    ref_i = 0
    data_i = 0
    residual_i = 0
    residual_pos = np.full(pos_data.shape, np.nan)
    residual_vel = np.full(vel_data.shape, np.nan)
    residual_att = np.full(att_data.shape, np.nan)
    residual_time = np.full(ref_time.shape, np.nan)
    while (data_i < np.size(data_time) and ref_i < np.size(ref_time)):
        if (np.abs(ref_time[ref_i] - data_time[data_i]) < 2e-3):
            residual_pos[residual_i, ...] = ref_pos_data[
                ref_i, ...] - pos_data[data_i, ...]
            residual_vel[residual_i, ...] = ref_vel_data[
                ref_i, ...] - vel_data[data_i, ...]
            residual_att[residual_i, ...] = ref_att_data[
                ref_i, ...] - att_data[data_i, ...]
            residual_time[residual_i] = ref_time[ref_i]
            '''  '''
            if ((residual_att[residual_i, 2]) > 180):
                residual_att[residual_i, 2] -= 360
            if ((residual_att[residual_i, 2]) < -180):
                residual_att[residual_i, 2] += 360
            ref_i += 1
            data_i += 1
            residual_i += 1
        elif (ref_time[ref_i] - data_time[data_i] > 0):
            data_i += 1
        else:
            ref_i += 1
    residual_pos = residual_pos[~np.isnan(residual_pos[..., 0]), ...]
    residual_vel = residual_vel[~np.isnan(residual_vel[..., 0]), ...]
    residual_att = residual_att[~np.isnan(residual_att[..., 0]), ...]
    residual_time = residual_time[~np.isnan(residual_time)]

    pos_mean = np.zeros([3, 3])
    vel_mean = np.zeros([3, 3])
    att_mean = np.zeros([3, 3])

    pos_mean[0, ...] = np.mean(residual_pos)
    vel_mean[0, ...] = np.mean(residual_vel)
    att_mean[0, ...] = np.mean(residual_att)

    pos_mean[1, ...] = np.std(residual_pos)
    vel_mean[1, ...] = np.std(residual_vel)
    att_mean[1, ...] = np.std(residual_att)

    pos_mean[2, ...] = np.sqrt(pos_mean[0, ...] * pos_mean[0, ...] +
                               pos_mean[1, ...] * pos_mean[1, ...])
    vel_mean[2, ...] = np.sqrt(vel_mean[0, ...] * vel_mean[0, ...] +
                               vel_mean[1, ...] * vel_mean[1, ...])
    att_mean[2, ...] = np.sqrt(att_mean[0, ...] * att_mean[0, ...] +
                               att_mean[1, ...] * att_mean[1, ...])
    # plot_residual(residual_time, residual_pos, residual_vel, residual_att,
    #               is_save_picture, save_path)
    # plot_imu_error(data_time, gyro_bias_data, acce_bias_data, gyro_scale_data,
                   acce_scale_data, is_save_picture, save_path)
    output_array = np.column_stack((residual_time, residual_pos, residual_vel,
                                    residual_att))
    np.savetxt(os.path.join(save_path, "residual.txt"), output_array, "%.5f")


if __name__ == "__main__":
    print("length of argv is %d" % len(sys.argv))
    if (len(sys.argv) < 3):
        print(
            "uasge: python ./evaluate.py result.txt reference.txt save_path start_time end_time is_save"
        )
        exit()
    if (len(sys.argv) == 4):
        compare(sys.argv[1], sys.argv[2], sys.argv[3])
    elif (len(sys.argv) == 5):
        compare(sys.argv[1], sys.argv[2], (sys.argv[3]), int(sys.argv[4]),
                int(sys.argv[5]))
    elif (len(sys.argv) == 7):
        compare(sys.argv[1], sys.argv[2], (sys.argv[3]), int(sys.argv[4]),
                (int(sys.argv[5])), bool(int(sys.argv[6])))
    else:
        print(
            "uasge: python ./evaluate.py result.txt reference.txt save_path start_time end_time is_save"
        )
