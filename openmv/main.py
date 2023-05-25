# Canny边缘检测:
#
# 这个例子展示了Canny边缘检测。
import sensor, image, time, os, tf, math, uos, gc
from pyb import UART,Timer
import micropython

# 串口
uart = UART(3, 115200, timeout_char = 1000) # 实例化

# 初始化sensor.
sensor.reset()

sensor.set_pixformat(sensor.GRAYSCALE) # or sensor.RGB565 设置图像颜色

sensor.set_framesize(sensor.QVGA) # or sensor.QVGA (or others) 设置图像像素大小

sensor.skip_frames(30) # 跳过一定帧 让新的设置生效

sensor.set_gainceiling(8) # 设置相机图像增益上限

# 跟踪FPS帧率
clock = time.clock()

# 设置使用模型
model = 1

threshold=100

#------------------------------------------------------------------------------------------

def compute_rows_centroids(img, max_diff=50, num_neighbours=2):
    centroids = []
    width, height = img.width(), img.height()
    for row in range(height):
        sum_x, count = 0, 0
        for col in range(width):
            if img.get_pixel(col, row) > 0:
                sum_x += col
                count += 1

        if count > 0:
            centroid_x = sum_x // count
            centroid = (centroid_x, row, count)
            if len(centroids) >= num_neighbours:  # 检查是否有足够的相邻质心
                neighbour_sum = 0
                num_valid_neighbours = 0
                for i in range(1, num_neighbours + 1):
                    if centroids[-i] is not None:
                        neighbour_sum += centroids[-i][0]
                        num_valid_neighbours += 1

                if num_valid_neighbours > 0:
                    avg_neighbour = neighbour_sum // num_valid_neighbours
                    if abs(centroid_x - avg_neighbour) <= max_diff:  # 和相邻质心偏差不大
                        centroids.append(centroid)
                    else:
                        centroids.append(None)
                else:
                    centroids.append(centroid)
            else:
                centroids.append(centroid)
        else:
            centroids.append(None)
    return centroids
# 定义一个函数，用于判断直线是否接近垂直
def is_vertical_line(line, tolerance=30):
    theta = line.theta()
    return abs(theta) <= tolerance or abs(theta - 180) <= tolerance

## 模型0：巡线
def line_patrol():

    clock.tick() # 追踪两个snapshots()之间经过的毫秒数.
    #img = sensor.snapshot() # 拍一张照片并返回图像。
    img = sensor.snapshot().replace(vflip=True,hmirror=True)

    # 二值化图像   #自适应二值化
    #img_statistics = img.get_statistics()
    #max_value = img_statistics.max()
    #uq_value = img_statistics.uq()
    ##print(max_value,uq_value)

    img = img.binary([(120,255)],invert=False)

    # 使用Canny边缘检测器 #threshold设置阈值
    img.find_edges(image.EDGE_CANNY, threshold=(70, 150))

    # 消噪
    #img.mean(2)

    # 池化
    img = img.mean_pool(2,2)

  # 计算每一行的质心
    rows_centroids = compute_rows_centroids(img)
    #print(rows_centroids)

    # 画出质心
    for centroid in rows_centroids:
        if centroid is not None:
            #img.draw_circle(centroid[0], centroid[1], math.floor(centroid[2]/5), color=255, fill = False)
            img.draw_rectangle(centroid[0], centroid[1], math.floor(centroid[2]/5), math.floor(centroid[2]/5),color=255)
    # 画出根据质心画出的线
    for i in range(len(rows_centroids) - 1, 0, -1):
        current_centroid = rows_centroids[i]
        prev_centroid = rows_centroids[i - 1]

        if current_centroid is not None and prev_centroid is not None:
            continue
            img.draw_line((current_centroid[0], current_centroid[1], prev_centroid[0], prev_centroid[1]), color=255,thickness=4)



    # 使用霍夫变换检测图像中的直线
    #lines = img.find_lines(threshold=1000, theta_margin=25, rho_margin=25)

    #for l in lines:
        #if is_vertical_line(l):
            #img.draw_line(l.line(), color=255)
            #print("rho: %d, theta: %d" % (l.rho(), l.theta()))

    ## 划线
    line = img.get_regression([(150,255)], robust = True)
    #(255,255):追踪的颜色范围：纯白色
    #robust = True:使用Theil-Sen线性回归算法
    if (line):
        rho_err = abs(line.rho())-img.width()/2
        if line.theta()>90:
            theta_err = line.theta()-180
        else:
            theta_err = line.theta()
        img.draw_line(line.line(), color = 127)
        if line.theta() >= 90:
            w = line.theta()-90 -90
        else:
            w = line.theta()+90 -90
        if w>=0:
            w = w+10
            command = "a1%03d" % (abs(w))
        else:
            w = w-10
            command = "a2%03d" % (abs(w))
        uart.write(command)
        print("command: "+command)

#------------------------------------------------------------------------------------------#

## 模型1：箭头检测
def identification_arrow():
    sensor.reset()                         # Reset and initialize the sensor.
    sensor.set_pixformat(sensor.RGB565)    # Set pixel format to RGB565
    sensor.set_framesize(sensor.B128X128)  # Set frame size to 128x128, whic is equal to model input
    sensor.set_windowing((128, 128))       # Set 128x128 window.
    sensor.skip_frames(time=2000)          # Let the camera adjust.


    net = None
    labels = None
    min_confidence = 0.7    # 需要根据实际情况设置置信度
    num_arrow = 20          # 检测到num_arrow次箭头后计算方向，20时根据识别情况，需要大约3-6s

    try:
        # load the model, alloc the model file on the heap if we have at least 64K free after loading
        net = tf.load("trained.tflite", load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - (64*1024)))
    except Exception as e:
        raise Exception('Failed to load "trained.tflite", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')

    try:
        labels = [line.rstrip('\n') for line in open("labels.txt")] # 包含了识别的种类和顺序
    except Exception as e:
        raise Exception('Failed to load "labels.txt", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')

    colors = [ # 用3种颜色显示不同的箭头
        (255,   0,   0),
        (  0, 255,   0),
        (255, 255,   0),
        (  0,   0, 255),
    ]

    clock = time.clock() #设置计时器记录帧率
    ARROW = [] #记录识别出的不同箭头
    clock.tick() #开始计时
    while(len(ARROW) <= num_arrow): # 识别到x次箭头后退出模型

        img = sensor.snapshot().replace(vflip=True,hmirror=True)
    # detect() returns all objects found in the image (splitted out per class already)
    # we skip class index 0, as that is the background, and then draw circles of the center
    # of our objects

        for i, detection_list in enumerate(net.detect(img, thresholds=[(math.ceil(min_confidence * 255), 255)])):
            if (i == 0): continue # background class
            if (len(detection_list) == 0): continue # no detections for this class
            else:
                if (i == 1): ARROW.append(i)
                elif (i == 2): ARROW.append(i)
                elif (i == 3): ARROW.append(i)

            #print("********** %s **********" % labels[i])
            for d in detection_list:
                [x, y, w, h] = d.rect()
                center_x = math.floor(x + (w / 2))
                center_y = math.floor(y + (h / 2))
                #print('x %d\ty %d' % (center_x, center_y))
                img.draw_circle((center_x, center_y, 12), color=colors[i], thickness=2)

    print(clock.fps(), "fps", end="\n\n")

    direct = max(ARROW, key=lambda v: ARROW.count(v))
    print("direction is: ",direct)
    
    uart.write("b%d" % (direct))
    
    if(direct == 1): pass # 向前走，具体时长和方向（防止被倒下立牌困住）需要实际测试
    elif(direct == 2): pass # 向左前走
    elif(direct == 3): pass # 向右前走

#------------------------------------------------------------------------------------------#

## 主循环
while(True):
    if model == 0:
        line_patrol()
    if model == 1:
        identification_arrow()


    print("now fps: "+ str(clock.fps())) # 注意:你的OpenMV摄像头的运行速度只有它的一半
