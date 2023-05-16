# Canny边缘检测:
#
# 这个例子展示了Canny边缘检测。
import sensor, image, time
from pyb import UART,Timer
import micropython

# 串口
uart = UART(3, 115200, timeout_char = 1000) # 实例化

# 初始化sensor.
sensor.reset()

sensor.set_pixformat(sensor.GRAYSCALE) # or sensor.RGB565 设置图像颜色

sensor.set_framesize(sensor.VGA) # or sensor.QVGA (or others) 设置图像像素大小

sensor.skip_frames(30) # 跳过一定帧 让新的设置生效

sensor.set_gainceiling(8) # 设置相机图像增益上限

# 跟踪FPS帧率
clock = time.clock()

# 设置使用模型
model = 0

#def a_func():
    #global command
    #uart.write(command+'xxx'+'\n')

## 定时器
#timer = Timer(4)
#timer.init(freq=2)
#timer.callback(lambda t: a_func())

#------------------------------------------------------------------------------------------#

## 模型0：巡线
def line_patrol():

    clock.tick() # 追踪两个snapshots()之间经过的毫秒数.
    img = sensor.snapshot() # 拍一张照片并返回图像。

    # 使用Canny边缘检测器 #threshold设置阈值
    img.find_edges(image.EDGE_CANNY, threshold=(60, 80))

    # 消噪
    img.mean(5)

    # 池化
    img = img.mean_pool(10,10)

    # 二值化图像   #自适应二值化
    img_statistics = img.get_statistics()
    max_value = img_statistics.max()
    uq_value = img_statistics.uq()
    #print(max_value,uq_value)

    img = img.binary([(uq_value,max_value)],invert=False)

    # 侵蚀
    #img.erode(1)

    # 扩张
    img.dilate(3)

    # 划线（霍夫变换）
    line = img.get_regression([(255,255)], robust = True)
    #(255,255):追踪的颜色范围：纯白色
    #robust = True:使用Theil-Sen线性回归算法
    if (line):#如果存在符合要求的直线
            rho_err = abs(line.rho())-img.width()/2
                #rho_err:负值直线在左侧正值在右侧若为水平线的中垂线则为0
                #line.rho():通过霍夫变换拟合出直线相对于原点的距离(像素数)(即r=xcosθ+ysinθ中的r)
                #img.width():以像素计图像的宽度
            if line.theta()>90:
                #line.theta():0-90 Y+半轴和直线的夹角,90-180 Y-半轴和直线的夹角
                theta_err = line.theta()-180
            else:
                theta_err = line.theta()
                #处理后:绝对值为直线同Y+轴的夹角,右正左负
            img.draw_line(line.line(), color = 127)
            # print(rho_err,line.rho(),line.magnitude(),line.theta(),theta_err)
            # line.magnitude():霍夫变换后所得直线的模

            # w：角度的，左偏为正，右偏为负,
            # print(str(line.theta()))
            if line.theta() >= 90:
                w = line.theta()-90 -90
            else:
                w = line.theta()+90 -90

            if w>=0:
                if -10<w<10:
                    command = 'a'+'1'+'0'+str(abs(w))
                else:
                    command = 'a'+'1'+str(abs(w))
            else:
                if -10<w<10:
                    command = 'a'+'0'+'0'+str(abs(w))
                else:
                    command = 'a'+'0'+str(abs(w))

            # 发16进制数据
            #code=[0x90,0x03,0x00,0x10,0x00,0x02,0xC5,0xCE]	# 要发送的数据列表
            #uart.write(bytes(code))	# 发送该HEX数据

            # 发字符串数据
            # a 1 039: a起始 0代表左，1代表右 039代表角度
            uart.write(command)
            print("command: "+command)

#------------------------------------------------------------------------------------------#

## 模型1：箭头检测
def identification_arrow():
    clock.tick() # 追踪两个snapshots()之间经过的毫秒数.
    img = sensor.snapshot() # 拍一张照片并返回图像。


#------------------------------------------------------------------------------------------#

## 主循环
while(True):
    if model == 0:
        line_patrol()
    if model == 1:
        identification_arrow()


    print("now fps: "+str(clock.fps())) # 注意:你的OpenMV摄像头的运行速度只有它的一半
