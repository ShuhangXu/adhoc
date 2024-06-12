import os
import moviepy.editor as mp

def reprocess_video(input_path, output_path):
    os.system(f"ffmpeg -i {input_path} -c:v copy -c:a copy {output_path}")

# Paths to original video and audio files
video1_path = "output_publisher1_camera1.avi"
video2_path = "output_publisher1_camera2.avi"
video3_path = "output_gaze.avi"
audio_path = "output_audio.wav"

# Paths to reprocessed video files
reprocessed_video1_path = "reprocessed_video1.avi"
reprocessed_video2_path = "reprocessed_video2.avi"
reprocessed_video3_path = "reprocessed_video3.avi"

# Reprocess the videos to ensure they have correct metadata
reprocess_video(video1_path, reprocessed_video1_path)
reprocess_video(video2_path, reprocessed_video2_path)
reprocess_video(video3_path, reprocessed_video3_path)

# Load the reprocessed video clips
video1 = mp.VideoFileClip(reprocessed_video1_path)
video2 = mp.VideoFileClip(reprocessed_video2_path)
video3 = mp.VideoFileClip(reprocessed_video3_path)

# Ensure all videos have the same duration
duration = min(video1.duration, video2.duration, video3.duration)
video1 = video1.subclip(0, duration)
video2 = video2.subclip(0, duration)
video3 = video3.subclip(0, duration)

# Resize the videos to ensure they fit together nicely
video3 = video3.resize(height=1080)

# Load the audio clip and trim to the duration of the videos
audio = mp.AudioFileClip(audio_path).subclip(0, duration)

# Combine the videos side by side in two rows (2 on top, 1 on bottom)
final_video = mp.CompositeVideoClip([
    mp.clips_array([[video1, video2], [video3, video3.set_opacity(0)]])
])

# Set the audio of the final video
final_video = final_video.set_audio(audio)

# Output the final video
output_path = "output_video.mp4"
final_video.write_videofile(output_path, codec="libx264")

print("Video processing completed!")
