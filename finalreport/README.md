# Architecture

## Abstract

## Task Analysis
### Patio1
In this section, the rover is required to follow a specific path with different colour and texture to the surrounding area while 4  turns are needed to make as well. After following the path, the rover needs to identify and go across a bridge. The third task is to recognise the gate and go through it and then stop.

To complete these tasks, Openmv requires visual recognition and pavement judgment capabilities to determine the optimal route. Three sensors are also needed to ensure correct crossing of the bridge and gate.

### Patio2
For Patio 2, the rover first needs to identify the direction of the arrow and move in the direction specified according to the identification result. In the next part, the rover needs to load and drop a ping pong ball in a specific basket. Finally, the rover enters a designated area and transmits student number, name and time information to the laptop at 433Hz radio signal.



To accompolish these tasks, a algorithm is needed to train the rover for identifying the arrow direction. Additionally, a robotic arm is equipped on the rover to load and deliver the ping pong ball. For the last task, we decided to use a wireless transceiver to realize communication capability. 

## Top System

## Sub Modules

### Chassis (Motors Control, Power, Battery)

### Motor Controller (H7XX)

### Ultrasonic sensors

### Gyroscope

### Servo

#### Robotic Arm

#### Gimbal Servo for OpenMV

### Main Controller (G474)

#### Fence Alignment

#### Rotate with a given angle

### OpenMV Sensors
#### Openmv Pavement Inspection

The rode to be identified is thin gravel pavement, while the surrounding road surface is flat slate pavement.
Judge the current direction of the rover according to the difference between two types of pavement and identify the edge.


##### Image Binarization



``` python
img = img.binary([(120,255)],invert=False)
```



This line of code is used to binarize the images. The meanings of every parameters are as follows:

- img = img.binary(): Performs binary operation on img and converts it into a binary image.

- invert=False: indicates that the binary image is not flipped. If set to True, then 1 becomes 0 and 0 becomes 1.

- So overall, what this line of code does is convert the pixels between 120 and 255 of the image to 1, producing a binary image. The brightness between 120 and 255 will be retained while the rest will turn black.



Such code performs image processing in OpenMV environment to produce a binary image. Binarization is an important step in image processing, which can be used in image segmentation, feature extraction and many other operations.

[![iShot_2023-03-11_12.44.10](https://github.com/ray24777/tdps2023/raw/main/openmv/assets/iShot_2023-03-11_12.44.10.png)](https://github.com/ray24777/tdps2023/blob/main/openmv/assets/iShot_2023-03-11_12.44.10.png)

#### Edge Detection

``` python
img.find_edges(image.EDGE_CANNY, threshold=(70, 150))
```

Canny operator is used to detect the edge of the road surface. The results show that the edge of the patrol road is dense and complex, while the edge of the normal road surface is sparse. This step needs to detect as many edges as possible, so a larger resolution is needed.

[![iShot_2023-03-11_12.41.36](https://github.com/ray24777/tdps2023/raw/main/openmv/assets/iShot_2023-03-11_12.41.36.png)](https://github.com/ray24777/tdps2023/blob/main/openmv/assets/iShot_2023-03-11_12.41.36.png)

#### Mean pooling

``` python
img = img.mean_pool(2,2)
```
Mean pooling is used to average every 2x2 pixels into one pixel to reduce the calculation workload of next step.

[![iShot_2023-03-11_12.43.18](https://github.com/ray24777/tdps2023/raw/main/openmv/assets/iShot_2023-03-11_12.43.18.png)](https://github.com/ray24777/tdps2023/blob/main/openmv/assets/iShot_2023-03-11_12.43.18.png)

#### Center of Mass Identification

``` python
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


**Draw center of mass**

Check through each row, and if the line outputs a trusted center of mass, then draw the center of mass with a square whose size is proportional to the number of visible pixels in that row.

This marks the image with a pure white path line, which encourages a high weight in the regression acquisition line.

**compute_rows_centroids(img, max_diff=50, num_neighbours=2)**

Calculate the centroid of each row in an image (centroid is the center of distribution of pixel values) and determine whether the centroid of adjacent rows is close enough. If close, keep, not close, discard.


The thorough logic is:
1. Check through each line of the image.
2. For each row, calculate the sum of x coordinates of the non-zero pixels sum_x and the number count.
3. When count> 0, calculates centroid_x = sum_x // count and generates a tuple representing the centroid (centroid_x, row, count).
4. Calculate the  average value of the nearest num_neighbours centroids as avg_neighbour.
5. If the difference of centroid_x for the current row is less than max_diff, it is considered close enough for the centroids of adjacent rows to be added into the centroids list.
6. If the centroids are not close enough to the centroids of adjacent rows, add None to the centroids list.
7. If the centroids list has fewer centroids than num_neighbours, add the centroids of the current row directly.
8. Make the above judgment and processing for each row, and finally return a list of centroids containing the centroid information of all retained rows.

So what this function does is to find the rows that are "close to each other" in the image and return the centroid information of these rows.

[![image-20230525170948979](https://github.com/ray24777/tdps2023/raw/main/openmv/assets/image-20230525170948979.png)](https://github.com/ray24777/tdps2023/blob/main/openmv/assets/image-20230525170948979.png)

#### Lineation



``` python
## 划线
    line = img.get_regression([(150,255)], robust = True)
    #150,255):追踪的颜色范围：150到255
    #robust = True:使用Theil-Sen线性回归算法
```



Operating Regression function to the binary image.

The operations are:

- img.get_regression(): Operating Regression function to the binary image。

- robust = True: Enable robust fit. This ignores outliers and results in more smooth and accurate regression results.

In general, what this line of code does is to perform a robust regression operation on the white pixels between 150 and 255 on the binary image, resulting in a smooth line or curve..

[![iShot_2023-03-11_12.45.57](https://github.com/ray24777/tdps2023/raw/main/openmv/assets/iShot_2023-03-11_12.45.57.png)](https://github.com/ray24777/tdps2023/blob/main/openmv/assets/iShot_2023-03-11_12.45.57.png)

#### Output



``` python
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

The principle of this code represents:

1. Calculate the distance rho_err and Angle theta_err between the straight line and the image center.
2. Draw a line on the image.
3. Calculate the Angle w of the steering gear according to the Angle theta of the line, if theta is greater than 90 degrees, then w=theta-90-90 degrees, otherwise w=theta+90-90 degrees.
4. Generate the steering gear control command according to the positive and negative values of w, in the format of a1xxx or a2xxx, where xxx is the absolute value of the Angle.
5. Send the control command to the steering gear through the serial port UART.
6. Print out the sent command.

Therefore, this code detects the straight line in the image, calculates the corresponding rotation Angle of the steering gear according to the Angle of the straight line, and sends control instructions to make the steering gear turn to this Angle so as to realize the closed-loop control based on vision. Possible applications are to turn a steering arm or vehicle in the direction of a straight line in the field of vision.



In this code:



- line.rho () and line.theta() obtain the rho and theta parameters of the line in the Hough transform, representing the position and direction of the line on the image.

- line.line () returns the start and end coordinates of a line, used to draw a line on the image.

- uart.write() sends data through the serial port.

- a1xxx and a2xxx are the control command formats sent to the steering gear.



In summary, this code implements a visual closed-loop control scheme based on line detection.

#### Other Images

[![iShot_2023-03-10_17.51.42](https://github.com/ray24777/tdps2023/raw/main/openmv/assets/iShot_2023-03-10_17.51.42.png)](https://github.com/ray24777/tdps2023/blob/main/openmv/assets/iShot_2023-03-10_17.51.42.png)

[![iShot_2023-03-10_17.49.55](https://github.com/ray24777/tdps2023/raw/main/openmv/assets/iShot_2023-03-10_17.49.55.png)](https://github.com/ray24777/tdps2023/blob/main/openmv/assets/iShot_2023-03-10_17.49.55.png)

#### Complete Code

``` python
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
#### Arrow Identification
This process is used to identify the arrow, when the arrow is identified as up, knock down the sign forward; When the arrow is identified to the left, knocked down the sign in direction of left forward; When the arrow is identified to the right, knock down the sign in direction of right forward.

#### Problem Analysis and Algorithm Selection
Since the arrow image on the logo is tiny and the arrow is not a conventional geometric figure, it is challenging to determine its precise position and identify the direction by normal algorithm, so we finally choose to deploy the quantized and trimmed lightweight artificial intelligence target detection model on OpenMV.
The model adopted FOMO (Faster Objects, More Objects) and was completed by Edge Impulse online model training test platform. FOMO is an early but small object recognition model that works by dividing input images into 8x8 resolution chunks (batches) and then classifying and summarizing each chunk individually. This network uses MobileNetV2 0.35 as the migration learning model.
Specifically, FOMO truncates the classic MobileNetV2 from the middle of feature extraction to obtain a feature map of size n x n x c (n is the width and height of the feature map, c is the number of channels of the feature map). For instance, if the input resolution is 128 x 128 and the expected feature map output size is 8 x 8, the input image resolution is reduced by a factor of 16. This means that the FOMO finds that layer in the MobileNetV2 network structure and truncates it, resulting in an 8 x 8 feature map.

[![img](https://camo.githubusercontent.com/f795472cc78c28b25214eab90c0c9dab4c9358ccda7483fe253d6517779220aa/68747470733a2f2f38343737313138382d66696c65732e676974626f6f6b2e696f2f7e2f66696c65732f76302f622f676974626f6f6b2d782d70726f642e61707073706f742e636f6d2f6f2f73706163657325324647456763436b34506b5335506136754261626c6425324675706c6f6164732532466769742d626c6f622d34363237393130383863653530356231373834356331663062666566396237376366613831653939253246326665646466362d686561746d6170322d62622e6a70673f616c743d6d6564696126746f6b656e3d66383666396166332d303264312d346632362d386137332d326239663063343665383331)](https://camo.githubusercontent.com/f795472cc78c28b25214eab90c0c9dab4c9358ccda7483fe253d6517779220aa/68747470733a2f2f38343737313138382d66696c65732e676974626f6f6b2e696f2f7e2f66696c65732f76302f622f676974626f6f6b2d782d70726f642e61707073706f742e636f6d2f6f2f73706163657325324647456763436b34506b5335506136754261626c6425324675706c6f6164732532466769742d626c6f622d34363237393130383863653530356231373834356331663062666566396237376366613831653939253246326665646466362d686561746d6170322d62622e6a70673f616c743d6d6564696126746f6b656e3d66383666396166332d303264312d346632362d386137332d326239663063343665383331)

In the output feature graph of size n x n x c, c represents that there are c categories, each layer is used to find the location of a classified object, and all the pixels of each layer represent whether there is an object of this classification in that location (namely, confidence). If we traverse the n x n x c feature map and find pixel coordinates whose confidence exceeds the set threshold, we can assume that there are objects to be detected in those coordinates and map them back to the original image at scale. Because truncated feature maps cannot accurately reflect corner coordinates of bounding box, FOMO will only output geometric centers of key feature maps to realize simple target detection.
We chose to use 128x128 resolution to balance the target resolution and frame rate at longer distances, and quantified the variables to int8 type to reduce the memory footprint of the model.
In the end, the model achieved 92.7% F1 value and 100% Precision in the test set.（添加F1公式）
It takes up only 30.3Kb of ROM and runs at about 10 frames.

#### Model Data Acquisition and Processing
On a sunny morning, at the test site, we used the acrylic plate we bought and pasted arrows of 10cm x 10cm in three different directions, ranging from 10cm-30cm in distance, 0-up 45 degrees in pitch Angle, plus or minus 10cm in left-right distance, etc. About 400 images with a resolution of 128x128 were captured using OpenMV camera to ensure that arrow images at different angles and distances could be captured.
After data cleaning, 96 upward arrow images, 105 left arrow images and 106 right arrow images were obtained as training data sets. The remaining 20% of the images are divided into the test set.
After comprehensive analysis, due to the weak computational power of OpenMV, it is unable to run the artificial intelligence model with good generalization of target size, such as YOLO, so  FOMO was chosen as the algorithm of machine learning.
Through the Edge Impulse online platform, all images are manually annotated after being uploaded. In this step, since the principle of FOMO is to divide into small square blocks and classify successively, there is a high probability that the trailing part of the left and right arrows cannot be correctly classified. So we chose to set only the tip of the arrow as the target area.
Because data enhancement would result in random reversal of the left and right arrows, it was not implemented. We chose 0.001 learning rate and 60 rounds of training as parameters for training, and obtained the following results:
[![image-20230519200330971](https://github.com/ray24777/tdps2023/raw/main/openmv/assets/image-20230519200330971.png)](https://github.com/ray24777/tdps2023/blob/main/openmv/assets/image-20230519200330971.png)
Eventually, after model tailoring and quantification, we possess a model file that can be deployed directly to OpenMV.

#### Model Deployment and Invocation
Arrow-recognition model has been functionally encapsulated and the following code was used for model deployment:

``` python
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



When this model is invoked, note the following:
1. Camera Settings are respecified when the function is called. So when the model call completes, you need to respecify the camera parameters.
2. The function should be enabled after the car reaches the designated position (i.e. right in front of the sign) and stops completely. The steering gear Angle can be adjusted to be horizontal or 45 degrees downtilt according to the actual test situation.
3. In order to avoid instantaneous error, the function will add the category of arrows of each detected frame to the list. When the length of the list reaches the specified number (tentatively 20), the function determines the final type through the mode of the list.

####  Model Recognition Demonstration

[![image-20230519200529926](https://github.com/ray24777/tdps2023/raw/main/openmv/assets/image-20230519200529926.png)](https://github.com/ray24777/tdps2023/blob/main/openmv/assets/image-20230519200529926.png)

[![image-20230519200612203](https://github.com/ray24777/tdps2023/raw/main/openmv/assets/image-20230519200612203.png)](https://github.com/ray24777/tdps2023/blob/main/openmv/assets/image-20230519200612203.png)

[![image-20230519200715197](https://github.com/ray24777/tdps2023/raw/main/openmv/assets/image-20230519200715197.png)](https://github.com/ray24777/tdps2023/blob/main/openmv/assets/image-20230519200715197.png)

[![image-20230519200737494](https://github.com/ray24777/tdps2023/raw/main/openmv/assets/image-20230519200737494.png)](https://github.com/ray24777/tdps2023/blob/main/openmv/assets/image-20230519200737494.png)
##### As can be seen, the model can identify the position and classify correctly for arrows of different angles and distances.

#### Gravel Path Tracking & Feedback

#### Communication with Main Controller

### PCB Design and Assembly

## System Integration

### Bottom Layer: Motor Driver, Power, Battery

### Middle Layer: PCB, Main Controller, Motor Controller, Ultrasonic Sensors, etc.

### Top Layer: OpenMV, Gimbal, Robotic Arm

## Field Test Results (To Be Done after testing)

### Patio 1

### Patio 2

## Conclusion and Future Improvments





