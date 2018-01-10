import image, math, pyb, sensor, struct, time
uart_baudrate = 115200
MAV_system_id = 1
MAV_component_id = 0x54
MAX_DISTANCE_SENSOR_enable = False
lens_mm = 2.8
lens_to_camera_mm = 22
sensor_w_mm = 3.984
sensor_h_mm = 2.952

threshold_index = 0 # 0 for red, 1 for green, 2 for blue

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green/blue things. You may wish to tune them...
thresholds = [(19, 98, 28, 96, -115, 124), # generic_red_thresholds
              (30, 100, -64, -8, -32, 32), # generic_green_thresholds
              (0, 30, 0, 64, -128, 0)] # generic_blue_thresholds

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(30)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking

x_res = 160
y_res = 120
f_x = (lens_mm / sensor_w_mm) * x_res
f_y = (lens_mm / sensor_h_mm) * y_res
c_x = x_res / 2
c_y = y_res / 2
h_fov = 2 * math.atan((sensor_w_mm / 2) / lens_mm)
v_fov = 2 * math.atan((sensor_h_mm / 2) / lens_mm)
def z_to_mm(z_translation, tag_size):
        return (((z_translation * 100) * tag_size) / 165) - lens_to_camera_mm
uart = pyb.UART(3, uart_baudrate, timeout_char = 1000)
packet_sequence = 0
def checksum(data, extra):
        output = 0xFFFF
        for i in range(len(data)):
                tmp = data[i] ^ (output & 0xFF)
                tmp = (tmp ^ (tmp << 4)) & 0xFF
                output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
        tmp = extra ^ (output & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
        return output

MAV_LANDING_TARGET_message_id = 149
MAV_LANDING_TARGET_min_distance = 1/100
MAV_LANDING_TARGET_max_distance = 10000/100
MAV_LANDING_TARGET_frame = 8
MAV_LANDING_TARGET_extra_crc = 200
def send_landing_target_packet(blob, w, h):
        global packet_sequence
        temp = struct.pack("<qfffffbb",
                                           0,
                                           (((blob.cx() / w) - 0.5) * h_fov)/2.25,
                                           (((blob.cy() / h) - 0.5) * v_fov)/1.65,
                                           0,   #int(z_to_mm(tag.z_translation(), tag_size) / -10),
                                           0.0,
                                           0.0,
                                           0,
                                           MAV_LANDING_TARGET_frame)
        temp = struct.pack("<bbbbb30s",
                                           30,
                                           packet_sequence & 0xFF,
                                           MAV_system_id,
                                           MAV_component_id,
                                           MAV_LANDING_TARGET_message_id,
                                           temp)
        temp = struct.pack("<b35sh",
                                           0xFE,
                                           temp,
                                           checksum(temp, MAV_LANDING_TARGET_extra_crc))
        packet_sequence += 1
        uart.write(temp)


clock = time.clock()

# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. "merge=True" merges all overlapping blobs in the image.

while(True):
        clock.tick()
        img = sensor.snapshot()
        for blob in img.find_blobs([thresholds[threshold_index]], pixels_threshold=40, area_threshold=20, merge=True):
              img.draw_rectangle(blob.rect())
              img.draw_cross(blob.cx(), blob.cy())
              send_landing_target_packet(blob, img.width(), img.height())
              #print("TRACK %f %f " % (blob.cx(), blob.cy()))
              #print("FPS %f" % clock.fps())
