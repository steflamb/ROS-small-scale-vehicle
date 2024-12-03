import matplotlib.pyplot as plt
import numpy as np
import cv2
import socket
import time
import math
import struct


def process_input(filename, width, length, folder):
    print(f"Filename: {filename}")
    print(f"Width: {width}")
    print(f"Length: {length}")
    print(f"Folder: {folder}")

def get_marker_id(frame,detector,id_in):
        corners, ids, _ = detector.detectMarkers(frame)
        center_marker = None
        corners_marker = None

        if ids is not None and id_in in ids:
            for i in range(len(ids)):
                if ids[i][0]==id_in:
                    c = corners[i][0]
                    center = (c[:, 0].mean(), c[:, 1].mean())
                    center_marker = center
                    corners_marker = c
                    break
        return center_marker, corners_marker

def get_markers(frame,detector):
        corners, ids, _ = detector.detectMarkers(frame)
        centers = {}
        corners_out = {}
        if ids is not None:
            for i in range(len(ids)):
                c = corners[i][0]
                center = (c[:, 0].mean(), c[:, 1].mean())
                centers[ids[i][0]] = center
                corners_out[ids[i][0]] = c
        return centers,corners_out

import json

def udp_format_data(items):
    # Prepare data dictionary with limited size
    data_dict = {
        "items_count": len(items),
        "items": []
    }
    for item in items:
        # Add each item's data with reduced name length and limited decimal places
        data_dict["items"].append({
            "object_id": item.get("object_id", 0),
            "name": item.get("name", "Unknown")[:10],  # Limit name to 10 chars
            "translation": [round(coord, 2) for coord in item.get("translation", (0.0, 0.0, 0.0))],
            "rotation": [round(angle, 2) for angle in item.get("rotation", (0.0, 0.0, 0.0))]
        })

    # Convert to JSON and encode to bytes
    data = json.dumps(data_dict).encode('utf-8')
    
    # Pad or truncate to ensure the total length is exactly 256 bytes
    data = data[:256].ljust(256, b'\x00')
    return data

def detect_objects(cap,perspective_matrix,perspective_matrix2,scale_1,scale_2,detector,objects_list,plot=False,stable_camera=True):
        port = 51001

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            print(f"Starting UDP broadcast loop on port {port} from local IP. Press Ctrl+C to stop.")
            frame_number=0
            try:
                while True:
                    ret, frame = cap.read()
                    frame = cv2.flip(frame, 0)
                    frame = cv2.flip(frame, 1)

                    if ret:
                        transformed_center={}
                        transformed_corners={}
                        scaled_coordinates={}
                        vector_side={}


                        # IF CAMERA IS NOT STABLE
                        if not stable_camera:
                            centers, _ = get_markers(frame, detector)
                            if all(id in centers for id in [1, 4, 2, 3]):
                                if plot:
                                    src_pts = np.array([centers[i] for i in [3, 4, 1, 2]], dtype=np.float32)
                                    dst_pts = np.array([[0, 0], [frame.shape[1], 0], [0, frame.shape[0]], [frame.shape[1], frame.shape[0]]], dtype=np.float32)

                                    perspective_matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)

                                src_pts = np.array([centers[i] for i in [3, 4, 1, 2]], dtype=np.float32)
                                dst_pts = np.array([[0, 0], [scale_1*200, 0], [0, scale_2*200], [scale_1*200,scale_2*200]], dtype=np.float32)
                                perspective_matrix2 = cv2.getPerspectiveTransform(src_pts, dst_pts)
                        ##########################

                        rectified_frame = cv2.warpPerspective(frame, perspective_matrix, (frame.shape[1], frame.shape[0]))

                        for id in objects_list.keys():
                            center_marker, corners_marker = get_marker_id(frame,detector,id)
                            if center_marker is not None:
                                center_marker_np = np.array([center_marker], dtype=np.float32).reshape(-1, 1, 2)
                                transformed_center_local = cv2.perspectiveTransform(center_marker_np, perspective_matrix2)[0][0]
                                x = scale_1 * transformed_center_local[0] / int(scale_1*200)
                                y = scale_2 * (1 - transformed_center_local[1] / int(scale_2*200))
                                corners = corners_marker
                                corners_np = np.array([corners], dtype=np.float32)
                                transformed_corners[id] = cv2.perspectiveTransform(corners_np, perspective_matrix)
                                vector_side[id] = transformed_corners[id][0][1] - transformed_corners[id][0][2]
                                angle = ((np.degrees(np.arctan2(vector_side[id][1], vector_side[id][0])))+90) % 360

                                x_shifted=x-(scale_1/2)
                                y_shifted=y-(scale_2/2)
                                scaled_coordinates[id]=[x_shifted,y_shifted,angle]

                                if plot:
                                    transformed_center[id] = cv2.perspectiveTransform(center_marker_np, perspective_matrix)[0][0]


                            else:
                                scaled_coordinates[id]=None
                                transformed_center[id]=None
                                transformed_corners[id]=None
                                vector_side[id]=None
                        items = []
                        for i,id in enumerate(objects_list.keys()):
                            if scaled_coordinates[id]!=None:
                                print(objects_list[id],scaled_coordinates[id][0],scaled_coordinates[id][1],scaled_coordinates[id][2])
                                dictionary={
                                    "object_id": i,
                                    "name": objects_list[id],
                                    "translation": (scaled_coordinates[id][0]*1000,scaled_coordinates[id][1]*1000,0.0),
                                    "rotation": (0.0, 0.0,math.radians(scaled_coordinates[id][2]))
                                }
                                items.append(dictionary)

                                if plot:
                                    for corner in transformed_corners[id][0]:
                                        cv2.circle(rectified_frame, (int(corner[0]), int(corner[1])), 15, (0, 0, 255), -1)
                                    cv2.circle(rectified_frame, (int(transformed_center[id][0]), int(transformed_center[id][1])), 15, (255, 0, 255), -1)
                                    cv2.arrowedLine(rectified_frame, (int(transformed_center[id][0]), int(transformed_center[id][1])),
                                                    (int(transformed_center[id][0] + vector_side[id][0]), int(transformed_center[id][1] + vector_side[id][1])),
                                                    (0, 255, 0), 2)

                                    text = f"{id}: {scaled_coordinates[id][0]:.2f} {scaled_coordinates[id][1]:.2f}"
                                    cv2.putText(rectified_frame, text, (int(transformed_center[id][0]), int(transformed_center[id][1]+50)), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2)

                        if plot:
                            cv2.circle(rectified_frame, (int(rectified_frame.shape[1]/2), int(rectified_frame.shape[0]/2)), 15, (255, 255, 255), -1)
                            cv2.imshow('Rectified Frame', rectified_frame)
                            cv2.waitKey(1)
                        message=udp_format_data(items)
                        broadcast_address = ('<broadcast>', port)
                        sock.sendto(message, broadcast_address)

                        frame_number+=1

            except KeyboardInterrupt:
                print("\nUDP broadcast loop stopped.")

def list_ports():
    is_working = True
    dev_port = 0
    working_ports = []
    available_ports = []
    while is_working:
        camera = cv2.VideoCapture(dev_port)
        if not camera.isOpened():
            is_working = False
            print("Port %s is not working." %dev_port)
        else:
            is_reading, img = camera.read()
            w = camera.get(3)
            h = camera.get(4)
            if is_reading:
                print("Port %s is working and reads images (%s x %s)" %(dev_port,h,w))
                working_ports.append(dev_port)
            else:
                print("Port %s for camera ( %s x %s) is present but does not reads." %(dev_port,h,w))
                available_ports.append(dev_port)
        dev_port +=1
    return available_ports,working_ports

# Main function
def main():
    _,port=list_ports()
    cap = cv2.VideoCapture(0)  # 0 represents the default webcam, change it if necessary

    x_cam,y_cam,fps=1080,1920,60
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, y_cam)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, x_cam)
    cap.set(cv2.CAP_PROP_FPS, fps)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    scale_1 = 4.85
    scale_2 = 2.5
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    frame_count=0
    found=False
    plot=True
    objects_list={
        10: "donkey",
        11: "barrier"
    }
    while frame_count < 50:
        ret, frame = cap.read()
        frame = cv2.flip(frame, 0)
        frame = cv2.flip(frame, 1)
        if not ret:
            print("No video")
            break
        frame_count=frame_count+1
        centers, _ = get_markers(frame, detector)
        print(centers)
        if all(id in centers for id in [1, 4, 2, 3]):
            found = True
            src_pts = np.array([centers[i] for i in [3, 4, 1, 2]], dtype=np.float32)
            dst_pts = np.array([[0, 0], [frame.shape[1], 0], [0, frame.shape[0]], [frame.shape[1], frame.shape[0]]], dtype=np.float32)

            perspective_matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)

            src_pts = np.array([centers[i] for i in [3, 4, 1, 2]], dtype=np.float32)
            dst_pts = np.array([[0, 0], [scale_1*200, 0], [0, scale_2*200], [scale_1*200,scale_2*200]], dtype=np.float32)

            perspective_matrix3 = cv2.getPerspectiveTransform(src_pts, dst_pts)

    if found==False:
        perspective_matrix=None
        _, ax = plt.subplots()
        ax.imshow(frame)
        plt.show()
        print("No corners detected! NO TRACKING")
    else:
        print("Start")
        detect_objects(cap,perspective_matrix,perspective_matrix3,scale_1,scale_2,detector,objects_list,plot)




# Entry point of the script
if __name__ == "__main__":
    main()

