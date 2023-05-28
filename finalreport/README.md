# Architecture

## Abstract

## Task Analysis
### Patio1
#### To successfully accomplish tasks 1.1 and 1.3, the rover is required to navigate by following a path marked with colorful tiles and accurately identify the location of a gate where it needs to come to a stop. To achieve this, OpenMV, the visual system integrated into the rover, must possess advanced capabilities in visual identification and pavement processing. By analyzing the visual data captured by its sensors, OpenMV can determine the optimal route to follow, ensuring that the rover stays on track and reaches the designated gate accurately. 

#### In the case of task 1.2, which involves locating and traversing a bridge, additional functionality is necessary. The rover needs to not only identify the presence of the bridge but also ensure proper alignment with both the bridge itself and its sidewall. To achieve this level of precision, three ultrasonic detectors are employed. These detectors emit sound waves that bounce off the bridge and its surroundings, allowing the rover to measure the distances and make necessary adjustments to align itself correctly. By leveraging the data gathered by the ultrasonic detectors, the rover can navigate across the bridge safely and confidently.

### Patio2
#### To effectively tackle Task 2.1, which entails the recognition of arrow shapes and their corresponding directions, specific adjustments must be made to the OpenMV camera system installed in patio1. These modifications may involve fine-tuning the camera's image processing algorithms, enhancing its ability to accurately detect and classify arrow shapes, and determining their orientations. By implementing these enhancements, the OpenMV camera can provide reliable and precise information regarding the arrow shapes and the directions they indicate.

#### Task 2.2 necessitates the successful completion of launching a tennis ball into a basket, which requires additional equipment in the form of a robotic arm integrated with a servo mechanism. The robotic arm plays a crucial role in precisely controlling the trajectory and force needed to propel the tennis ball towards the designated basket. By leveraging the servo mechanism, the robotic arm can achieve the necessary range of motion and accuracy to ensure the successful execution of this task. The coordination between the robotic arm and the servo mechanism allows for consistent and reliable ball launching, contributing to the overall efficiency of the operation.

#### In Task 2.3, effective communication takes precedence. The primary objective revolves around establishing seamless communication between the rover and a laptop computer. This is accomplished through the utilization of a wireless transceiver, which enables the transmission of messages and data between the rover and the laptop. The wireless transceiver serves as the link that facilitates real-time and reliable communication, allowing the rover to transmit vital information, receive instructions, and provide status updates to the laptop computer.
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

#### Arrow Identification
##### This process is used to identify the arrow, when the arrow is identified as up, knock down the sign forward; When the arrow is identified to the left, knocked down the sign in direction of left forward; When the arrow is identified to the right, knock down the sign in direction of right forward.

#### Problem Analysis and Algorithm Selection
##### Since the arrow image on the logo is tiny and the arrow is not a conventional geometric figure, it is challenging to determine its precise position and identify the direction by normal algorithm, so we finally choose to deploy the quantized and trimmed lightweight artificial intelligence target detection model on OpenMV.
##### The model adopted FOMO (Faster Objects, More Objects) and was completed by Edge Impulse online model training test platform. FOMO is an early but small object recognition model that works by dividing input images into 8x8 resolution chunks (batches) and then classifying and summarizing each chunk individually. This network uses MobileNetV2 0.35 as the migration learning model.
##### Specifically, FOMO truncates the classic MobileNetV2 from the middle of feature extraction to obtain a feature map of size n x n x c (n is the width and height of the feature map, c is the number of channels of the feature map). For instance, if the input resolution is 128 x 128 and the expected feature map output size is 8 x 8, the input image resolution is reduced by a factor of 16. This means that the FOMO finds that layer in the MobileNetV2 network structure and truncates it, resulting in an 8 x 8 feature map.

[![img](https://camo.githubusercontent.com/f795472cc78c28b25214eab90c0c9dab4c9358ccda7483fe253d6517779220aa/68747470733a2f2f38343737313138382d66696c65732e676974626f6f6b2e696f2f7e2f66696c65732f76302f622f676974626f6f6b2d782d70726f642e61707073706f742e636f6d2f6f2f73706163657325324647456763436b34506b5335506136754261626c6425324675706c6f6164732532466769742d626c6f622d34363237393130383863653530356231373834356331663062666566396237376366613831653939253246326665646466362d686561746d6170322d62622e6a70673f616c743d6d6564696126746f6b656e3d66383666396166332d303264312d346632362d386137332d326239663063343665383331)](https://camo.githubusercontent.com/f795472cc78c28b25214eab90c0c9dab4c9358ccda7483fe253d6517779220aa/68747470733a2f2f38343737313138382d66696c65732e676974626f6f6b2e696f2f7e2f66696c65732f76302f622f676974626f6f6b2d782d70726f642e61707073706f742e636f6d2f6f2f73706163657325324647456763436b34506b5335506136754261626c6425324675706c6f6164732532466769742d626c6f622d34363237393130383863653530356231373834356331663062666566396237376366613831653939253246326665646466362d686561746d6170322d62622e6a70673f616c743d6d6564696126746f6b656e3d66383666396166332d303264312d346632362d386137332d326239663063343665383331)

##### In the output feature graph of size n x n x c, c represents that there are c categories, each layer is used to find the location of a classified object, and all the pixels of each layer represent whether there is an object of this classification in that location (namely, confidence). If we traverse the n x n x c feature map and find pixel coordinates whose confidence exceeds the set threshold, we can assume that there are objects to be detected in those coordinates and map them back to the original image at scale. Because truncated feature maps cannot accurately reflect corner coordinates of bounding box, FOMO will only output geometric centers of key feature maps to realize simple target detection.
##### We chose to use 128x128 resolution to balance the target resolution and frame rate at longer distances, and quantified the variables to int8 type to reduce the memory footprint of the model.
##### In the end, the model achieved 92.7% F1 value and 100% Precision in the test set.（添加F1公式）
##### It takes up only 30.3Kb of ROM and runs at about 10 frames.

#### Model Data Acquisition and Processing
##### On a sunny morning, at the test site, we used the acrylic plate we bought and pasted arrows of 10cm x 10cm in three different directions, ranging from 10cm-30cm in distance, 0-up 45 degrees in pitch Angle, plus or minus 10cm in left-right distance, etc. About 400 images with a resolution of 128x128 were captured using OpenMV camera to ensure that arrow images at different angles and distances could be captured.
After data cleaning, 96 upward arrow images, 105 left arrow images and 106 right arrow images were obtained as training data sets. The remaining 20% of the images are divided into the test set.
##### After comprehensive analysis, due to the weak computational power of OpenMV, it is unable to run the artificial intelligence model with good generalization of target size, such as YOLO, so  FOMO was chosen as the algorithm of machine learning.
##### Through the Edge Impulse online platform, all images are manually annotated after being uploaded. In this step, since the principle of FOMO is to divide into small square blocks and classify successively, there is a high probability that the trailing part of the left and right arrows cannot be correctly classified. So we chose to set only the tip of the arrow as the target area.
##### Because data enhancement would result in random reversal of the left and right arrows, it was not implemented. We chose 0.001 learning rate and 60 rounds of training as parameters for training, and obtained the following results:
[![image-20230519200330971](https://github.com/ray24777/tdps2023/raw/main/openmv/assets/image-20230519200330971.png)](https://github.com/ray24777/tdps2023/blob/main/openmv/assets/image-20230519200330971.png)
##### Eventually, after model tailoring and quantification, we possess a model file that can be deployed directly to OpenMV.

#### Model Deployment and Invocation

##### Arrow-recognition model has been functionally encapsulated and the following code was used for model deployment:

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



##### When this model is invoked, note the following:
##### 1. Camera Settings are respecified when the function is called. So when the model call completes, you need to respecify the camera parameters.
##### 2. The function should be enabled after the car reaches the designated position (i.e. right in front of the sign) and stops completely. The steering gear Angle can be adjusted to be horizontal or 45 degrees downtilt according to the actual test situation.
##### 3. In order to avoid instantaneous error, the function will add the category of arrows of each detected frame to the list. When the length of the list reaches the specified number (tentatively 20), the function determines the final type through the mode of the list.

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





