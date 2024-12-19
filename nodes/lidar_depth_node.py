#!/usr/bin/env python3

from io import BytesIO
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import numpy as np
import requests
from PIL import Image as PILImage


video_source = rospy.get_param("lidar_source")



def lidar_depth_node():
    print("Starting lidar depth node")

    r = requests.get('http://team10.local:8890/lidar', stream=True)
    # print(type(r))
    print("Connected to the car")

    if r.encoding is None:
        r.encoding = 'utf-8'
    
    rospy.init_node("lidar_depth_node", anonymous=True)
    pub_img = rospy.Publisher("lidar/image", Image,queue_size=10)
    #the topic "lidar/depth_map" contains the flattened intrinsic matrix followed directly by the flattened depth map
    pub_lidar_data = rospy.Publisher("lidar/depth_map", Float64MultiArray, queue_size=10)
    bridge = CvBridge()

    intrinsic_matrix = None
    depth_map = None

    matrix_bytes = b""
    image_bytes = b""
    reading_matrix = False
    reading_image = False
    matrix_length = 0
    image_length = 0
    matrix_id = None
    image_id = None

    matrix_saved = False

    line_number = 1
    for line in r.iter_lines(decode_unicode=None):
        # print(f"LINE {line_number}: {line} ENF OF LINE {len(line)}")
        line_number +=1
        if rospy.is_shutdown():
            break
        if line:
            if "octet_stream".encode() in line:
                #Reached the beginning of a new message
                if len(matrix_bytes)>0:
                    #we process the data gathered from the previous message
                    print("\n\nREACHED END OF DATA BLOCK\n\n")
                    # matrix_bytes = matrix_bytes[1:]
                    matrix_bytes = matrix_bytes.replace(b'\\r', b'\r') 
                    # image_bytes = image_bytes[1:]
                    image_bytes = image_bytes.replace(b'\\r', b'\r')
                    image_bytes = image_bytes.replace(b'\\n', b'\n')

                    if not matrix_id == image_id:
                    # or not matrix_length==len(matrix_bytes) or not image_length==len(image_bytes):
                        print(f"Declared length of matrix: {matrix_length}\nLength read: {len(matrix_length)}")
                        print(f"Declared length of image: {image_length}\nLength read: {len(image_bytes)}")
                        matrix_bytes = b""
                        image_bytes = b""
                        matrix_length = 0
                        image_length = 0
                        matrix_id = None
                        image_id = None
                        print("ID does not match\n\n")
                        line_number = 1
                        continue

                    
                    try:
                        if not matrix_saved:    #FOR ONLY USING THE FIRST MATRIX
                            intrinsic_matrix = np.frombuffer(matrix_bytes)
                            matrix_saved = True
                        # print(intrinsic_matrix)
                    except Exception as e:
                        print("\n\nSOMETHING WENT WRONG PROCESSING THE MATRIX")
                        print(f"Declared length of matrix: {matrix_length}\nLength read: {len(matrix_length)}")
                        print(e)
                        matrix_bytes = b""
                        image_bytes = b""
                        matrix_length = 0
                        image_length = 0
                        matrix_id = None
                        image_id = None
                        line_number = 1
                        continue


                    try:
                        # print("processing image")
                        f = BytesIO(image_bytes)
                        pil_img = PILImage.open(f)
                        # cv2_img = np.rot90(np.array(pil_img))
                        cv2_img = np.array(pil_img)
                        # print(cv2_img.shape)

                        image_message = bridge.cv2_to_imgmsg(cv2_img, encoding="mono8")
                        pub_img.publish(image_message)

                        depth_map = (cv2_img.flatten()/255)*10
                        # depth_map = (cv2_img.flatten()/255)/10
                        lidar_data = Float64MultiArray()
                        lidar_data.data = np.concatenate((intrinsic_matrix, depth_map))
                        pub_lidar_data.publish(lidar_data)
                        print("PROCESSING SUCCESSFUL")

                    except Exception as e:
                        print("SOMETHING WENT WRONG PROCESSING THE IMAGE")
                        print(f"Declared length of image: {image_length}\nLength read: {len(image_bytes)}")
                        print(e)
                        matrix_bytes = b""
                        image_bytes = b""
                        matrix_length = 0
                        image_length = 0
                        matrix_id = None
                        image_id = None
                        line_number = 1
                        continue

                    matrix_bytes = b""
                    image_bytes = b""
                    matrix_length = 0
                    image_length = 0
                    matrix_id = None
                    image_id = None
                    line_number = 1

                line = line.decode()
                reading_matrix = True
                parts = line.split(";")
                # print(f"Parts: {parts}")
                matrix_id = parts[2].split(": ")[1]
                # print(f"Matrix ID is {matrix_id}")
                matrix_length = parts[1].split(": ")[1]
                # print(f"Matrix length is {matrix_length}")
            elif "jpeg".encode() in line:
                line = line.decode()
                reading_image = True
                parts = line.split(";")
                # print(f"Parts: {parts}")
                image_id = parts[2].split(": ")[1]
                # print(f"Image ID is {image_id}")
                image_length = parts[1].split(": ")[1]
                # print(f"Image length is {image_length}")
            elif "--".encode() in line and reading_matrix:
                matrix_bytes = matrix_bytes + line.split("--".encode())[0]
                reading_matrix = False
            elif "--".encode() in line and reading_image:
                image_bytes = image_bytes + line.split("--".encode())[0]
                reading_image = False       
            elif reading_matrix:
                matrix_bytes = matrix_bytes + line
            elif reading_image:
                image_bytes = image_bytes + line
        else:
            print("Line was none")

    print("[LIDAR CLIENT]: QUIT")







if __name__ == '__main__':
    try:
        lidar_depth_node()
    except rospy.ROSInterruptException:
        pass