import cv2

# Paths to the input videos
video_path_1 = 'output_publisher1_camera1.avi'
video_path_2 = 'output_publisher1_camera2.avi'
video_path_3 = 'output_gaze.avi'

# Open the video files
cap1 = cv2.VideoCapture(video_path_1)
cap2 = cv2.VideoCapture(video_path_2)
cap3 = cv2.VideoCapture(video_path_3)

# Get properties from the first video (assuming all have same properties)
fps = cap1.get(cv2.CAP_PROP_FPS)
frame_width = int(cap1.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap1.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, fps, (1920, frame_height + frame_height + frame_height))

while cap1.isOpened() and cap2.isOpened() and cap3.isOpened():
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()
    ret3, frame3 = cap3.read()

    if not ret1 or not ret2 or not ret3:
        break

    # Resize frames if necessary to match width
    if frame1.shape[1] != 1920:
        frame1 = cv2.resize(frame1, (1920, frame1.shape[0]))
    if frame2.shape[1] != 1920:
        frame2 = cv2.resize(frame2, (1920, frame2.shape[0]))
    if frame3.shape[1] != 1920:
        frame3 = cv2.resize(frame3, (1920, frame3.shape[0]))

    # Align frames vertically
    combined_frame = cv2.vconcat([frame1, frame2, frame3])

    # Write the combined frame
    out.write(combined_frame)

# Release everything
cap1.release()
cap2.release()
cap3.release()
out.release()
cv2.destroyAllWindows()

print("Video combination complete. Output saved as 'output.avi'")
