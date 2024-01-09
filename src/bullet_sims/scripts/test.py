import numpy as np
import matplotlib.pyplot as plt

# 生成时间范围
t = np.linspace(0, 2 * np.pi, 1000)

# 计算y1、y2、y3的值
y1 = np.sin(t)
y2 = np.sin(2 * t)
y3 = np.sin(3 * t)

# 创建第一个图像对象及子图
fig1, axes1 = plt.subplots(3, 1, figsize=(8, 10))

# 绘制第一个图像的三个子图
axes1[0].plot(t, y1, label='y1 = sin(t)')
axes1[0].set_ylabel('Amplitude')
axes1[0].legend()

axes1[1].plot(t, y2, label='y2 = sin(2t)')
axes1[1].set_ylabel('Amplitude')
axes1[1].legend()

axes1[2].plot(t, y3, label='y3 = sin(3t)')
axes1[2].set_xlabel('t')
axes1[2].set_ylabel('Amplitude')
axes1[2].legend()

# 创建第二个图像对象及子图
fig2, axes2 = plt.subplots(3, 1, figsize=(8, 10))

# 绘制第二个图像的三个子图
axes2[0].plot(t, y1, label='y1 = sin(t)')
axes2[0].set_ylabel('Amplitude')
axes2[0].legend()

axes2[1].plot(t, y2, label='y2 = sin(2t)')
axes2[1].set_ylabel('Amplitude')
axes2[1].legend()

axes2[2].plot(t, y3, label='y3 = sin(3t)')
axes2[2].set_xlabel('t')
axes2[2].set_ylabel('Amplitude')
axes2[2].legend()

# 创建第三个图像对象及子图
fig3, axes3 = plt.subplots(3, 1, figsize=(8, 10))

# 绘制第三个图像的三个子图
axes3[0].plot(t, y1, label='y1 = sin(t)')
axes3[0].set_ylabel('Amplitude')
axes3[0].legend()

axes3[1].plot(t, y2, label='y2 = sin(2t)')
axes3[1].set_ylabel('Amplitude')
axes3[1].legend()

axes3[2].plot(t, y3, label='y3 = sin(3t)')
axes3[2].set_xlabel('t')
axes3[2].set_ylabel('Amplitude')
axes3[2].legend()

# 显示图像
plt.show()
