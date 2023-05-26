[TOC]

# openmv 路面巡线

用于巡线路面为稀碎石子路面，周围路面为平整石板路面。

根据用于巡线路面和巡线路面周围路面纹理不同来判断当前小车前进的方向。

## 二值化图像

```python
img = img.binary([(120,255)],invert=False)
```

这行代码的作用是对图像img进行二值化处理。各个参数的意义如下:

- img = img.binary():表示对img进行二值化操作,将其转化为二值图像。
- invert=False:表示不翻转二值图像。如果置为True,则1会变为0,0变为1。
- 所以整体来说,这行代码的作用是将img图像的120-255之间的像素转化为1,产生一个二值图像。120-255之间的亮度会被保留,其余部分会变成黑色。

该代码在OpenMV环境下进行图像处理,产生一个二值化的图像。二值化是图像处理的一个很重要的步骤,后续可以用于图像分割、特征提取等许多操作。

<img src="./assets/iShot_2023-03-11_12.44.10.png" alt="iShot_2023-03-11_12.44.10" style="zoom:50%;" />



## 边缘检测

```
img.find_edges(image.EDGE_CANNY, threshold=(70, 150))
```

通过canny算子对路面进行边缘检测，得到的结果为巡线路面边缘密集复杂，正常路面边缘稀疏。这一步需要检测到尽量多的边缘，因此使用较大的分辨率。

<img src="./assets/iShot_2023-03-11_12.41.36.png" alt="iShot_2023-03-11_12.41.36" style="zoom:50%;" />

## 池化

```
img = img.mean_pool(2,2)
```

使用均值池化，每2x2大小像素均值为一个像素，降低下一步计算开销。

<img src="./assets/iShot_2023-03-11_12.43.18.png" alt="iShot_2023-03-11_12.43.18" style="zoom:50%;" />

## 质心计算并画出

```python
# 计算每一行的质心
rows_centroids = compute_rows_centroids(img)

# 画出质心
for centroid in rows_centroids:
        if centroid is not None:
            img.draw_rectangle(centroid[0], centroid[1], math.floor(centroid[2]/5), math.floor(centroid[2]/5),color=255)

# 定义一个函数，用于计算每行的质心，并且如果一行的质心与其他行的质心相差不大，则将其合并，相差大则不合并
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
```

**画出质心**

遍历每一行，如果这一行输出可信质心存在，那么就用正方形画出质心，正方形的大小正比于这一行的可见像素点数量。

这样就在图片上标注了一个纯白色路径线，这在回归获取直线中会得到很高的权重。

**compute_rows_centroids(img, max_diff=50, num_neighbours=2)**

在一幅图像中计算每行的质心(质心就是像素值分布中心),并判断相邻几行的质心是否足够接近。如果接近则保留,不接近则丢弃。

具体逻辑是:

1. 遍历图像的每行
2. 对于每行,计算非零像素的x坐标之和sum_x和个数count
3. 如果count>0,则计算该行的质心x坐标centroid_x = sum_x // count,并生成一个表示质心的元组(centroid_x, row, count)
4. 如果centroids列表已经有num_neighbours个质心了,则计算最近num_neighbours个质心的平均值avg_neighbour
5. 如果当前行的质心centroid_x与avg_neighbour的偏差<=max_diff,则认为与相邻行质心足够接近,将其添加到centroids列表
6. 否则认为与相邻行质心不够接近,在centroids列表添加None
7. 如果centroids列表不足num_neighbours个质心,则直接添加当前行的质心
8. 对每行进行上述判断和处理,最终返回centroids列表,其中包含所有被保留的行质心信息

所以这个函数的作用是在图像中找出位于“相互接近”的行,并返回这些行的质心信息。

![image-20230525170948979](./assets/image-20230525170948979.png)

## 划线

```python
## 划线
    line = img.get_regression([(150,255)], robust = True)
    #150,255):追踪的颜色范围：150到255
    #robust = True:使用Theil-Sen线性回归算法
```

在二值图像img上进行Regression(回归)运算。

- img.get_regression():在图像img上进行Regression运算。
- robust = True:启用鲁棒拟合。这会忽略离群点和异常值,得到更加平滑和准确的回归结果。

所以整体来说,这行代码的作用是:在二值图像img上的150-255之间的白色像素上,进行一个鲁棒的回归运算,得到一条平滑的直线或曲线。



<img src="./assets/iShot_2023-03-11_12.45.57.png" alt="iShot_2023-03-11_12.45.57" style="zoom:50%;" />

## 输出

```python
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
```

这段代码的作用是:

1. 计算直线line与图像中心的距离rho_err和角度theta_err
2. 在图像上绘制直线line
3. 根据直线的角度theta计算舵机的角度w,如果theta>=90度,则w=theta-90-90度,否则w=theta+90-90度
4. 根据w的正负,生成舵机控制指令command,格式为a1xxx或a2xxx,xxx为角度的绝对值
5. 通过串口uart向舵机发送该控制指令
6. 打印输出发送的指令

所以,这段代码检测图像中的直线,根据直线的角度计算相应的舵机转动角度,并发送控制指令使舵机转到该角度,从而实现基于视觉的闭环控制。可能的应用是使舵机控制的机械臂或车辆随着视野中直线的方向转动。

这段代码中:

- line.rho()和line.theta()获取直线在Hough变换中的rho和theta参数,代表直线在图像上的位置和方向
- line.line()返回直线的起点和终点坐标,用于在图像上绘制直线
- uart.write()通过串口发送数据
- a1xxx和a2xxx是发送给舵机的控制指令格式

所以总的说,这段代码实现了一种基于直线检测的视觉闭环控制方案。





## 其他图片

![iShot_2023-03-10_17.51.42](./assets/iShot_2023-03-10_17.51.42.png)

![iShot_2023-03-10_17.49.55](./assets/iShot_2023-03-10_17.49.55.png)

## 完整代码

```python
### 模型0：巡线------------------------------------------------------------------------------------------
# 定义一个函数，用于计算每行的质心，并且如果一行的质心与其他行的质心相差不大，则将其合并，相差大则不合并
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

# 定义巡线主函数
def line_patrol():

    # 追踪两个snapshots()之间经过的毫秒数.
    clock.tick()

    # 从sensor中获取图像，180度翻转
    img = sensor.snapshot().replace(vflip=True,hmirror=True)

    # 二值化图像
    img = img.binary([(120,255)],invert=False)

    # 使用Canny边缘检测器 #threshold设置阈值
    img.find_edges(image.EDGE_CANNY, threshold=(70, 150))

    # 消噪
    #img.mean(2)

    # 池化
    img = img.mean_pool(2,2)

    # 计算每一行的质心
    rows_centroids = compute_rows_centroids(img)

    # 画出质心
    for centroid in rows_centroids:
        if centroid is not None:
            img.draw_rectangle(centroid[0], centroid[1], math.floor(centroid[2]/5), math.floor(centroid[2]/5),color=255)

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

```



# 箭头识别

用于识别箭头，当识别到箭头为朝上，向前撞倒立牌；箭头为朝左，向左前方撞倒立牌；箭头为朝右，向右前方撞倒立牌。

## 问题分析与算法选择

由于立牌上的箭头图像较小，且箭头并非常规几何图形，难以通过常规算法确定其精准位置并裁剪识别方向，故我们最终选择在OpenMV上部署经过量化、裁剪后的轻量人工智能目标检测模型。

模型采用FOMO (Faster Objects, More Objects)，通过Edge Impulse在线模型训练测试平台完成。FOMO是一种早期但小巧的目标识别模型，通过将输入图片切分为8x8分辨率的小块(batch)，然后对每个小块分别进行分类并进行汇总。该网络将MobileNetV2 0.35作为迁移学习模型。

具体而言，FOMO将经典的MobileNetV2从特征提取的中间截断，得到一个大小为n x n x c (n是特征图的宽高，c 是特征图的通道数) 的特征图。例如，如果输入分辨率是128 x 128且期望特征图输出大小为8 x 8，则相当于把输入图片分辨率降低了16倍。这意味着FOMO会从MobileNetV2网络结构中找到该层进行截断，最终得到8 x 8的特征图。

![img](https://84771188-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2FGEgcCk4PkS5Pa6uBabld%2Fuploads%2Fgit-blob-462791088ce505b17845c1f0bfef9b77cfa81e99%2F2feddf6-heatmap2-bb.jpg?alt=media&token=f86f9af3-02d1-4f26-8a73-2b9f0c46e831)

在输出的大小为n x n x c的特征图中，c代表了有c个分类，每一层都用来寻找一个分类的物体的位置，每层的所有像素就代表了在该位置是否存在该分类的物体（即置信度）。如果我们遍历了这个n x n x c的特征图并找到了置信度超过设置阈值的像素坐标，我们便能认为这些坐标中存在待检测的物体，也就能按照缩放比例将其映射回原图。由于截断的特征图并不能准确反映出bounding box的角点坐标，FOMO只会输出关键特征图的几何中心，以实现简单的目标检测。

我们选择使用128x128分辨率以平衡较远距离时的目标分辨率和帧率，并将变量量化为int8类型以减少模型的内存占用。

最终，该模型在测试集中实现了92.7%的F1值(F1 value)，并达到了100%的精准率(Precision)。**（这里可以把F1值/精准率的公式加上）**其占用ROM仅为30.3Kb，运行帧率大约为10帧。

## 模型数据采集与处理

在晴天上午，于测试场地，我们使用购买的亚克力板，上贴10cm x 10cm的三种不同朝向的箭头，从距离10cm-30cm，俯仰角0-向上45度，左右间距正负10cm等不同机位，通过OpenMV相机采集了约400张分辨率为128x128的图片，以确保不同角度，不同距离的箭头图片都能得到采集。

在经过数据清洗后，得到朝上箭头图片96张，朝左箭头图片105张，朝右箭头图片106张作为训练数据集。剩下20%的图片被划分到测试集。

经过综合分析，由于OpenMV算力较弱，无法运行YOLO等对目标大小泛化性良好的人工智能模型，因此我们选择了FOMO作为机器学习的算法。

通过Edge Impulse在线平台，所有图片在被上传后进行人工标注。在这一步，由于FOMO的原理是划分成正方形小块依次分类，所以左右箭头的拖尾部分有很大概率无法被正确分类。因此我选择了只将箭头尖端部分设置为目标区域。

由于数据增强会导致左右箭头出现随机颠倒的情况，故没有进行。我们选择以0.001的学习率，60轮训练作为参数进行训练，获得了如下的效果：

![image-20230519200330971](./assets/image-20230519200330971.png)

最后，经过模型裁剪与量化，我们获得了可直接部署到OpenMV上的模型文件。

## 模型部署与调用

我们将箭头识别模型的调用进行了函数封装，使用以下代码进行模型部署：

```python
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

        img = sensor.snapshot()
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
    if(direct == 1): pass # 向前走，具体时长和方向（防止被倒下立牌困住）需要实际测试
    elif(direct == 2): pass # 向左前走
    elif(direct == 3): pass # 向右前走
```

当您需要调用该模型时，请注意以下几点：

1. 当函数被调用后，相机设置会被重新指定。因此当模型调用完成后，需要您重新指定相机参数。
2. 该函数应该在小车到达指定位置（即立牌正前方）并彻底停下后启用，可根据实际测试情况调整舵机角度为平视或45度下倾。
3. 为了避免瞬时误差，函数会将检测到的每一帧的箭头的类别添加到列表中，当列表长度达到指定数目（暂定20）后，函数通过列表的众数来确定最终的类型。

## 模型效果展示

![image-20230519200529926](./assets/image-20230519200529926.png)

![image-20230519200612203](./assets/image-20230519200612203.png)

![image-20230519200715197](./assets/image-20230519200715197.png)

![image-20230519200737494](./assets/image-20230519200737494.png)

可以看到，模型对不同角度和远近的箭头都能比较好地识别出位置并正确分类。

## 需要改进的地方

1. 回传给主控消息。（需要实地测试计算）
2. 有一定可能将暗角识别为箭头（相当于多识别了箭头），但实地场景可能不会出现此类情况。如果需要改进，可以多拍箭头占比比较大的照片重新训练。（很快，主要麻烦的是给数据打标签。。。）
3. 置信度，检测总时间等参数还能进一步调整。
4. 舵机的俯仰角感觉不用调整，但也需要测试。
5. 我这个亚克力板比较小，但场地的测试亚克力板立不起来。。。
6. 具体摆放立牌的位置不确定，当天的天气情况也不确定。准备再试试数据增强，拉一下曝光亮度对比度之类的。
