import numpy as np
import cv2

def main():
    data = np.genfromtxt("/home/jacob/Desktop/output.csv", skip_header=1, delimiter=",")
    frame = data[:,0].astype(int)
    detected = data[:,1].astype(int)

    video = cv2.VideoCapture("/home/jacob/Desktop/output.mp4")

    if not video.isOpened():
        print("Fejl: Kan ikke åbne video")
        return

    total_frames = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
    current_frame = 0

    while True:
        # hop til frame
        video.set(cv2.CAP_PROP_POS_FRAMES, current_frame)
        ret, img = video.read()

        if not ret:
            break

        # vis info
        if current_frame < len(detected):
            print(f'Frame: {current_frame}, detected: {bool(detected[current_frame])}')

            if detected[current_frame]:
                cv2.putText(img, "DETECTED", (50,50),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1, (0,0,255), 2)

        cv2.imshow("video", img)

        key = cv2.waitKey(0)

        if key == ord('q'):
            break
        elif key == ord('d'):  # frem
            current_frame = min(current_frame + 1, total_frames - 1)
        elif key == ord('a'):  # tilbage
            current_frame = max(current_frame - 1, 0)

    video.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()