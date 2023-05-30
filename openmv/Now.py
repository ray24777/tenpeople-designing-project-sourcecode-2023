import sensor, image, time, os, tf, math, uos, gc
from pyb import UART,Timer
import micropython
uart = UART(3, 115200, timeout_char = 1000)
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(30)
sensor.set_gainceiling(8)
clock = time.clock()
model = 0
threshold=100
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
			if len(centroids) >= num_neighbours:
				neighbour_sum = 0
				num_valid_neighbours = 0
				for i in range(1, num_neighbours + 1):
					if centroids[-i] is not None:
						neighbour_sum += centroids[-i][0]
						num_valid_neighbours += 1
				if num_valid_neighbours > 0:
					avg_neighbour = neighbour_sum // num_valid_neighbours
					if abs(centroid_x - avg_neighbour) <= max_diff:
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
def is_vertical_line(line, tolerance=30):
	theta = line.theta()
	return abs(theta) <= tolerance or abs(theta - 180) <= tolerance
def line_patrol():
	clock.tick()
	img = sensor.snapshot().replace(vflip=True,hmirror=True)
	img = img.binary([(120,255)],invert=False)
	img.find_edges(image.EDGE_CANNY, threshold=(70, 150))
	img = img.mean_pool(2,2)
	rows_centroids = compute_rows_centroids(img)
	for centroid in rows_centroids:
		if centroid is not None:
			img.draw_rectangle(centroid[0], centroid[1], math.floor(centroid[2]/5), math.floor(centroid[2]/5),color=255)
	for i in range(len(rows_centroids) - 1, 0, -1):
		current_centroid = rows_centroids[i]
		prev_centroid = rows_centroids[i - 1]
		if current_centroid is not None and prev_centroid is not None:
			continue
			img.draw_line((current_centroid[0], current_centroid[1], prev_centroid[0], prev_centroid[1]), color=255,thickness=4)
	line = img.get_regression([(150,255)], robust = True)
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
def identification_arrow():
	sensor.reset()
	sensor.set_pixformat(sensor.RGB565)
	sensor.set_framesize(sensor.B128X128)
	sensor.set_windowing((128, 128))
	sensor.skip_frames(time=2000)
	net = None
	labels = None
	min_confidence = 0.7
	num_arrow = 20
	try:
		net = tf.load("trained.tflite", load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - (64*1024)))
	except Exception as e:
		raise Exception('Failed to load "trained.tflite", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')
	try:
		labels = [line.rstrip('\n') for line in open("labels.txt")]
	except Exception as e:
		raise Exception('Failed to load "labels.txt", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')
	colors = [
		(255,   0,   0),
		(  0, 255,   0),
		(255, 255,   0),
		(  0,   0, 255),
	]
	clock = time.clock()
	ARROW = []
	clock.tick()
	while(len(ARROW) <= num_arrow):
		img = sensor.snapshot().replace(vflip=True,hmirror=True)
		for i, detection_list in enumerate(net.detect(img, thresholds=[(math.ceil(min_confidence * 255), 255)])):
			if (i == 0): continue
			if (len(detection_list) == 0): continue
			else:
				if (i == 1): ARROW.append(i)
				elif (i == 2): ARROW.append(i)
				elif (i == 3): ARROW.append(i)
			for d in detection_list:
				[x, y, w, h] = d.rect()
				center_x = math.floor(x + (w / 2))
				center_y = math.floor(y + (h / 2))
				img.draw_circle((center_x, center_y, 12), color=colors[i], thickness=2)
	print(clock.fps(), "fps", end="\n\n")
	direct = max(ARROW, key=lambda v: ARROW.count(v))
	print("direction is: ",direct)
	if(direct == 1):
		uart.write('b1')
	elif(direct == 2):
		uart.write('b2')
	elif(direct == 3):
		uart.write('b3')
rp = None
while rp == None :
	rp = uart.read(5)
	print(rp)
	if rp == b'task1':
		model = 0
		print("task1")
	elif rp == b'task2':
		model = 1
		print("task2")
	else:
		rp = None
while(True):
	if model == 0:
		line_patrol()
	if model == 1:
		identification_arrow()
	print("now fps: "+ str(clock.fps()))