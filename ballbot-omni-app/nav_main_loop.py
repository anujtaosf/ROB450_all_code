from camera_module import get_cam_data # EDIT THIS LINE WHEN FUNCTION IS COMPLETE
from april_tag_navigation import navigate_ballbot


cam = get_cam_data()

try:
    while True:
        color_img, depth_frame = cam.get_frames()
        navigate_ballbot()
        lin_vel, ang_vel = process_camera_input(color_img, depth_frame)
        print(f"Linear: {lin_vel:.2f}, Angular: {ang_vel:.2f}")
        # send commands to ballbot
        
finally:
    cam.stop()