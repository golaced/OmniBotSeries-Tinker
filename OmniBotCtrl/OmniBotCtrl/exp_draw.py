import matplotlib.pyplot as plt
import numpy as np
 
# 定义x的范围
x = np.linspace(-0.2, 0.2, 1000)
# 计算对应的exp(x)值
x_norm=np.linalg.norm(x)
#y = np.exp(-x_norm)
#y = -np.exp(-np.abs(x)*100)
y = np.abs(x)
# 绘制图像
plt.plot(x, y)
# 设置标题和轴标签
plt.title('Plot of exp(x)')
plt.xlabel('x')
plt.ylabel('exp(x)')
# 显示图例
plt.legend(['exp(x)'])
# 显示网格
plt.grid()
# 显示图像
plt.show()

