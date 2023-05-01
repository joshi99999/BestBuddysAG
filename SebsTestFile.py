import numpy as np
import cv2

def getMatrix1(x, y):
    A = np.array([[x[0,0], x[0,1], 1, 0, 0, 0, -x[0,0]*y[0,0], -x[0,1]*y[0,0]],
                  [x[1,0], x[1,1], 1, 0, 0, 0, -x[1,0]*y[1,0], -x[1,1]*y[1,0]],
                  [x[2,0], x[2,1], 1, 0, 0, 0, -x[2,0]*y[2,0], -x[2,1]*y[2,0]],
                  [x[3,0], x[3,1], 1, 0, 0, 0, -x[3,0]*y[3,0], -x[3,1]*y[3,0]],
                  [0, 0, 0, x[0,0], x[0,1], 1, -x[0,0]*y[0,1], -x[0,1]*y[0,1]],
                  [0, 0, 0, x[1,0], x[1,1], 1, -x[1,0]*y[1,1], -x[1,1]*y[1,1]],
                  [0, 0, 0, x[2,0], x[2,1], 1, -x[2,0]*y[2,1], -x[2,1]*y[2,1]],
                  [0, 0, 0, x[3,0], x[3,1], 1, -x[3,0]*y[3,1], -x[3,1]*y[3,1]]], dtype=np.float32)
    b = np.concatenate((y[:,0], y[:,1]))
    x = np.linalg.inv(A) @ b
    M = np.array([[x[0], x[1], x[2]],
                  [x[3], x[4], x[5]],
                  [x[6], x[7], 1]], dtype=np.float32)
    return M

def getMatrix2(x, y, a):
    A = np.array([[x[0,0]*y[0,1,1], x[0,1]*y[0,1,1], y[0,1,1], -x[0,0]*y[0,1,0], -x[0,1]*y[0,1,0], -y[0,1,0], -x[0,0]*(y[0,0,0]*y[0,1,1]-y[0,0,1]*y[0,1,0]), -x[0,1]*(y[0,0,0]*y[0,1,1]-y[0,0,1]*y[0,1,0])],
                  [x[1,0]*y[1,1,1], x[1,1]*y[1,1,1], y[1,1,1], -x[1,0]*y[1,1,0], -x[1,1]*y[1,1,0], -y[1,1,0], -x[1,0]*(y[1,0,0]*y[1,1,1]-y[1,0,1]*y[1,1,0]), -x[1,1]*(y[1,0,0]*y[1,1,1]-y[1,0,1]*y[1,1,0])],
                  [x[2,0]*y[2,1,1], x[2,1]*y[2,1,1], y[2,1,1], -x[2,0]*y[2,1,0], -x[2,1]*y[2,1,0], -y[2,1,0], -x[2,0]*(y[2,0,0]*y[2,1,1]-y[2,0,1]*y[2,1,0]), -x[2,1]*(y[2,0,0]*y[2,1,1]-y[2,0,1]*y[2,1,0])],
                  [x[3,0]*y[3,1,1], x[3,1]*y[3,1,1], y[3,1,1], -x[3,0]*y[3,1,0], -x[3,1]*y[3,1,0], -y[3,1,0], -x[3,0]*(y[3,0,0]*y[3,1,1]-y[3,0,1]*y[3,1,0]), -x[3,1]*(y[3,0,0]*y[3,1,1]-y[3,0,1]*y[3,1,0])],
                  [x[4,0]*y[4,1,1], x[4,1]*y[4,1,1], y[4,1,1], -x[4,0]*y[4,1,0], -x[4,1]*y[4,1,0], -y[4,1,0], -x[4,0]*(y[4,0,0]*y[4,1,1]-y[4,0,1]*y[4,1,0]), -x[4,1]*(y[4,0,0]*y[4,1,1]-y[4,0,1]*y[4,1,0])],
                  [x[5,0]*y[5,1,1], x[5,1]*y[5,1,1], y[5,1,1], -x[5,0]*y[5,1,0], -x[5,1]*y[5,1,0], -y[5,1,0], -x[5,0]*(y[5,0,0]*y[5,1,1]-y[5,0,1]*y[5,1,0]), -x[5,1]*(y[5,0,0]*y[5,1,1]-y[5,0,1]*y[5,1,0])],
                  [a[0,0], a[0,1], 1, 0, 0, 0, -a[0,0]*a[1,0], -a[0,1]*a[1,0]],
                  [0, 0, 0, a[0,0], a[0,1], 1, -a[0,0]*a[1,1], -a[0,1]*a[1,1]]], dtype=np.float64)
    b = np.concatenate((y[:,0,0]*y[:,1,1]-y[:,0,1]*y[:,1,0], a[1]), dtype=np.float64)
    o = np.linalg.inv(A) @ b
    M = np.array([[o[0], o[1], o[2]],
                  [o[3], o[4], o[5]],
                  [o[6], o[7], 1]], dtype=np.float32)
    return M
    
    
org = np.zeros((500,500), dtype=np.uint8)

org[50:500:50,:] = 255
org[:,50:500:50] = 255


pts2 = np.float32([[15,50],[600,20],[45,560],[540,490]])
pts1 = np.float32([[0,0],[500,0],[0,500],[500,500]])

M = cv2.getPerspectiveTransform(pts1,pts2)
print(M)
M1 = getMatrix1(pts1,pts2)

x = np.array([[50,50],[100,50],[150,50],[50,100],[100,100],[150,100]], dtype=np.float32)
y = np.zeros((6,2,2), dtype=np.float32)

for i in range(3):
    w = M @ np.concatenate((x[i], [1]))
    y[i,0] = y[i+3,0] = w[[0,1]] / w[2]
    w = M @ np.concatenate((x[i+3], [1]))
    y[i,1] = y[i+3,1] = w[[0,1]] / w[2] - y[i,0]

w = M @ [200,200,1]
a = np.array([[200,200],w[[0,1]]/w[2]], dtype=np.float32)

M2 = getMatrix2(x,y,a)
print(M2)

warped = cv2.warpPerspective(org,M1,(500,500))

color = cv2.cvtColor(src=warped, code=cv2.COLOR_GRAY2BGR)

w = M @ [150,100,1]
print(w[[0,1]]/w[2])

for i in range(3):
    color = cv2.circle(img=color, center=y[i,0].astype(np.int32), radius=3, color=(0,0,255), thickness=-1)
    color = cv2.circle(img=color, center=(y[i,0]+y[i,1]).astype(np.int32), radius=3, color=(0,0,255), thickness=-1)

color = cv2.circle(img=color, center=(a[1]).astype(np.int32), radius=3, color=(0,255,0), thickness=-1)

w = M2 @ np.concatenate((x[1],[1]))
g = w[[0,1]]/w[2] - a[1]
w = M2 @ np.concatenate((x[2],[1]))
h = w[[0,1]]/w[2] - a[1]

print("g: " + str(g) + ", h: " + str(h))
print(g / h)

for i in range(6):
    w = M2 @ np.concatenate((x[i],[1]))
    print(np.cross(w[[0,1]]/w[2] - y[i,0], y[i,1]))
    color = cv2.circle(img=color, center=(w[[0,1]]/w[2]).astype(np.int32), radius=3, color=(255,0,0), thickness=-1)

cv2.imshow("Original", org)
cv2.imshow("Verzerrt", color)
cv2.waitKey()