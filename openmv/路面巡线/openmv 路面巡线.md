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

<img src="/Users/tlpa/Documents/GitHub/tenpeople-designing-project-sourcecode-2023/openmv/assets/iShot_2023-03-11_12.44.10.png" alt="iShot_2023-03-11_12.44.10" style="zoom:50%;" />



## 边缘检测

```
img.find_edges(image.EDGE_CANNY, threshold=(70, 150))
```

通过canny算子对路面进行边缘检测，得到的结果为巡线路面边缘密集复杂，正常路面边缘稀疏。这一步需要检测到尽量多的边缘，因此使用较大的分辨率。

<img src="/Users/tlpa/Documents/GitHub/tenpeople-designing-project-sourcecode-2023/openmv/assets/iShot_2023-03-11_12.41.36.png" alt="iShot_2023-03-11_12.41.36" style="zoom:50%;" />

## 池化

```
img = img.mean_pool(2,2)
```

使用均值池化，每2x2大小像素均值为一个像素，降低下一步计算开销。

<img src="/Users/tlpa/Documents/GitHub/tenpeople-designing-project-sourcecode-2023/openmv/assets/iShot_2023-03-11_12.43.18.png" alt="iShot_2023-03-11_12.43.18" style="zoom:50%;" />

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

![image-20230525170948979](/Users/tlpa/Documents/GitHub/tenpeople-designing-project-sourcecode-2023/openmv/assets/image-20230525170948979.png)

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



<img src="/Users/tlpa/Documents/GitHub/tenpeople-designing-project-sourcecode-2023/openmv/assets/iShot_2023-03-11_12.45.57.png" alt="iShot_2023-03-11_12.45.57" style="zoom:50%;" />

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

![iShot_2023-03-10_17.51.42](/Users/tlpa/Documents/GitHub/tenpeople-designing-project-sourcecode-2023/openmv/assets/iShot_2023-03-10_17.51.42.png)

![iShot_2023-03-10_17.49.55](/Users/tlpa/Documents/GitHub/tenpeople-designing-project-sourcecode-2023/openmv/assets/iShot_2023-03-10_17.49.55.png)

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



设置自动增益白点关，设置亮度固定值。灰度图像，二值化，边缘检测，池化，质心计算，画质心线，输出



