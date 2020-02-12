from moviepy.editor import ImageSequenceClip
from moviepy import editor as mpy
import glob

if __name__ == "__main__":
    image_list = glob.glob('bag_frames_color_classified/*.jpg')
    image_list.sort()
    # print(image_list)
    clip = mpy.ImageSequenceClip(image_list, fps=10)
    clip.write_videofile("/tmp/result.mp4")
