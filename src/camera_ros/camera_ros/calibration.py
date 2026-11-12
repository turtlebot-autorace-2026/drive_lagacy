import rclpy, cv2, os, glob, pickle, time
import numpy as np
from rclpy.node import Node

class CameraCalibrationNode(Node):
    def __init__(self):
        super().__init__('camera_calibration_node')
        self.image_save_interval = 1.0  # seconds
        self.last_save_time = time.time()
        self.image_counter = 0
        BASE_DIR = os.path.dirname(os.path.abspath(__file__))
        self.save_dir = os.path.join(BASE_DIR, 'calibration_images')
        os.makedirs(self.save_dir, exist_ok=True)
        self.get_logger().info("Capture images every second... Press 'q' in OpenCV window to stop and calibrate.")
        self.stop_requested = False

        # OpenCV camera
        self.cap = cv2.VideoCapture(0)
        #self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        #self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera.")
            self.stop_requested = True

    def capture_loop(self):
        while rclpy.ok() and not self.stop_requested:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("Failed to grab frame from camera")
                continue

            cv2.imshow('Live Capture - Press q to stop', frame)
            key = cv2.waitKey(1) & 0xFF

            if time.time() - self.last_save_time >= self.image_save_interval:
                filename = os.path.join(self.save_dir, f'calib_{self.image_counter:03d}.jpg')
                cv2.imwrite(filename, frame)
                self.get_logger().info(f"Saved: {filename}")
                self.last_save_time = time.time()
                self.image_counter += 1

            if key == ord('q'):
                self.get_logger().info("Stopping capture and calibrating...")
                self.stop_requested = True

        self.cap.release()
        cv2.destroyAllWindows()


def calibrate_camera(image_folder):
    CHECKERBOARD = (7, 9)
    square_size = 18.2  # 21mm
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objpoints = []
    imgpoints = []
    objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * square_size
    images = glob.glob(os.path.join(image_folder, '*.jpg'))
    gray = None

    for fname in images:
        img = cv2.imread(fname)
        if img is None: continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(
            gray, CHECKERBOARD,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)
            cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
            cv2.imshow('Calibration Result', img)
            cv2.waitKey(200)
    cv2.destroyAllWindows()

    if gray is None or len(objpoints) < 1:
        print("Calibration failed: No valid chessboard patterns found.")
        return

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )
    print("\nCamera matrix:\n", mtx)
    print("\nDistortion coefficients:\n", dist)

    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(BASE_DIR, 'camera_calibration.pkl'), 'wb') as f:
        pickle.dump({
            'camera_matrix': mtx,
            'dist_coeffs': dist,
            'rvecs': rvecs,
            'tvecs': tvecs
        }, f)
    print(f"\nCalibration data saved to {os.path.join(BASE_DIR, 'camera_calibration.pkl')}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrationNode()
    try:
        node.capture_loop()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
    if node.stop_requested:
        calibrate_camera(node.save_dir)


if __name__ == '__main__':
    main()

