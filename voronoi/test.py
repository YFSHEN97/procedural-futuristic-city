import cv2
import numpy as np
import math
import time
from voronoi import Voronoi2D


if __name__ == "__main__":

    time1 = time.time()

    s = 255 / math.sqrt(2)
    h = 500
    w = 500
    texture = Voronoi2D(10, 1.0, "euclidean")

    F123 = [[texture.evaluate(i/h, j/w, 3) for j in range(w)] for i in range(h)]

    def f1_func(i, j):
        i = int(i)
        j = int(j)
        return min(int(F123[i][j][2] * s), 255)

    def f2_func(i, j):
        i = int(i)
        j = int(j)
        return min(int(F123[i][j][1] * s), 255)

    def f3_func(i, j):
        i = int(i)
        j = int(j)
        return min(int(F123[i][j][0] * s), 255)


    F1 = np.fromfunction(np.vectorize(f1_func), (h, w)).astype(np.uint8)
    F2 = np.fromfunction(np.vectorize(f2_func), (h, w)).astype(np.uint8)
    F3 = np.fromfunction(np.vectorize(f3_func), (h, w)).astype(np.uint8)

    cv2.imwrite("F1.jpg", F1)
    cv2.imwrite("F2.jpg", F2)
    cv2.imwrite("F3.jpg", F3)

    time2 = time.time()
    print("Time taken: {} seconds".format(time2 - time1))

