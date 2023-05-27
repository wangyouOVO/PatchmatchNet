import datasets.data_io as dd
import cv2
depthMap = r'F:\DATA\dtu\dtu\scan33\depth_est\00000016.pfm'
ref_depth_est = dd.read_map(depthMap).squeeze(2)
print(ref_depth_est)
ref_depth_est = ref_depth_est/1500
print(ref_depth_est)
ref_depth_est = cv2.resize(ref_depth_est,(600,400))
cv2.imshow("ref_depth", ref_depth_est)
cv2.moveWindow("ref_depth", 50, 50)
cv2.waitKey(0)