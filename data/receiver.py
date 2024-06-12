import socket
import cv2
import pickle
import struct
import threading
import numpy as np  # Add this import

def handle_client(client_socket, addr, output_prefix):
    print(f'Connection from: {addr}')
    
    data = b""
    payload_size = struct.calcsize("Q")
    
    # Set up video writers
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out1 = cv2.VideoWriter(f'{output_prefix}_output1.avi', fourcc, 30.0, (640, 480))
    out2 = cv2.VideoWriter(f'{output_prefix}_output2.avi', fourcc, 30.0, (640, 480))
    
    while True:
        while len(data) < payload_size:
            packet = client_socket.recv(4*1024)  # 4K
            if not packet:
                break
            data += packet
        if not data:
            break

        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack("Q", packed_msg_size)[0]

        while len(data) < msg_size:
            data += client_socket.recv(4*1024)

        frame_data = data[:msg_size]
        data = data[msg_size:]

        frame1_encoded, frame2_encoded = pickle.loads(frame_data, fix_imports=True, encoding="bytes")

        # Decode frames
        frame1 = cv2.imdecode(np.frombuffer(frame1_encoded, np.uint8), cv2.IMREAD_COLOR)
        frame2 = cv2.imdecode(np.frombuffer(frame2_encoded, np.uint8), cv2.IMREAD_COLOR)

        # Write the frames to the video files
        out1.write(frame1)
        out2.write(frame2)

        # Display the frames (optional)
        cv2.imshow(f'{output_prefix} Video 1', frame1)
        cv2.imshow(f'{output_prefix} Video 2', frame2)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    client_socket.close()
    out1.release()
    out2.release()
    cv2.destroyAllWindows()

# Socket creation
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host_ip = '192.168.0.139'  # IP address of the receiver computer
port = 9999
socket_address = (host_ip, port)

# Bind the socket to the address and listen for connections
server_socket.bind(socket_address)
server_socket.listen(5)
print("Listening at:", socket_address)

# Handle multiple connections
client_count = 0
while True:
    if client_count == 2:
        break
    client_socket, addr = server_socket.accept()
    client_count += 1
    output_prefix = f'client{client_count}'
    client_thread = threading.Thread(target=handle_client, args=(client_socket, addr, output_prefix))
    client_thread.start()
