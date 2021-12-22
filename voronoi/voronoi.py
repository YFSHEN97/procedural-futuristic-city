import random
import math


# a heap (priority queue) storing elements in descending order
# elements must be comparable using <, =, > operators
class MaxHeap:
    def __init__(self):
        self.heap = [] # array used to store elements
        # given 0-based indexing in self.heap
        # for i-th element, its children are 2i + 1 and 2i + 2
        # for i-th element, its parent is (i-1) // 2

    # return current size of the heap (number of elements)
    def size(self):
        return len(self.heap)

    # insert item into the max heap
    def insert(self, item):
        self.heap.append(item)
        i = len(self.heap) - 1
        # next, keep swapping with its parent until its parent is bigger (or equal)
        while (i-1)//2 >= 0 and self.heap[(i-1)//2] < self.heap[i]:
            self.swap((i-1)//2, i)
            i = (i-1)//2

    # get the element at the top of the max heap
    def peek(self):
        return self.heap[0] if self.heap else float("inf")
    
    # remove the element at the top of the max heap and return it
    def pop(self):
        # first swap the first and last element of heap array
        self.swap(0, -1)
        # next we pop the last element
        ret = self.heap.pop()
        # finally we swap the new top of the heap into place
        i = 0
        while 2*i + 2 < len(self.heap) and (self.heap[i] < self.heap[2*i+1] or self.heap[i] < self.heap[2*i+2]):
            if self.heap[2*i+1] == max(self.heap[2*i+1], self.heap[2*i+2]): # left child is larger
                self.swap(i, 2*i+1)
                i = 2*i + 1
            else: # right child is larger
                self.swap(i, 2*i+2)
                i = 2*i + 2
        # although 2*i + 2 is out of range, we still need to check whether 2*i + 1 is out of range
        if 2*i + 1 < len(self.heap):
            if self.heap[i] < self.heap[2*i+1]:
                self.swap(i, 2*i+1)
        return ret
        
    def swap(self, i, j): # helper function to swap i and j-th elements in the heap
        temp = self.heap[i]
        self.heap[i] = self.heap[j]
        self.heap[j] = temp



# Voronoi2D provides an interface to create seamless 2D Voronoi textures
class Voronoi2D:
    # the __init__ method creates a S * S square grid of cells
    # each cell contains 1 seed point (feature point)
    # size: the dimensions of the cells (must be at least 5)
    # randomness: control how much the seed point positions can deviate from cell centers
    # mode: how to compute the distance metric (supports "euclidean", "manhattan", "chebyshev")
    def __init__(self, size, randomness=1.0, mode="euclidean"):
        self.size = size
        self.randomness = randomness
        self.mode = mode
        self.points = [[None for _ in range(size)] for _ in range(size)]
        for r in range(size):
            for c in range(size):
                self.points[r][c] = (
                    c + 0.5 + randomness * (random.random() - 0.5),
                    r + 0.5 + randomness * (random.random() - 0.5))


    # compute distance between the input point p (with row/col r, c)
    # and the seed point in cell (i, j)
    # i and j can be negative or out of bounds
    # in which case we do "wrap-around"
    def dist(self, p, r, c, i, j):
        rr = i % self.size
        cc = j % self.size
        x, y = self.points[rr][cc]
        if i < 0:
            dy = self.size - y + p[1]
        elif i >= self.size:
            dy = y + self.size - p[1]
        else:
            dy = abs(p[1] - y)
        if j < 0:
            dx = self.size - x + p[0]
        elif j >= self.size:
            dx = x + self.size - p[0]
        else:
            dx = abs(p[0] - x)
        if self.mode == "euclidean":
            return math.sqrt(dx * dx + dy * dy)
        elif self.mode == "manhattan":
            return dx + dy
        else:
            return max(dx, dy)



    # evaluate the 2D Voronoi texture function at texture coordinates (u, v)
    # u and v are both within range [0, 1]
    # n specifies the order of the texture function
    # i.e. n = 1 means finding the 1st closest neighbor, 2 means the 2nd closest, etc.
    # returns a list of n values, giving [Fn(u, v), Fn-1(u, v), ..., F1(u, v)]
    # for 2D texture, the maximum value of n supported is 3
    def evaluate(self, u, v, n):
        # we use a max heap to keep track of the n smallest distances we've found so far
        # each time we see a new distance, do the following:
        #   if current size of heap < n, insert new element, and continue
        #   if new element is greater than or equal to the top of max heap, we ignore it
        #   otherwise, pop the top of heap, and insert the new element
        # after processing all points, the heap contains the n smallest, in descending order

        # to speed up computations, we only look at neighboring cells
        # 20 cells (5 * 5 box except 4 corners and center/self)
        # i.e. see below (XX means 4 corner cells we ignore, UV is center/self)
        #   __ __ __ __ __
        #   XX|__|__|__|XX
        #   __|__|__|__|__
        #   __|__|UV|__|__
        #   __|__|__|__|__
        #   XX|__|__|__|XX
        #
        # to make sure the textures are seamless, need to consider wrap-arounds

        x = u * self.size # x coordinate of (u, v)
        y = v * self.size # y coordinate of (u, v)
        r = int(y)        # row index of the cell that (u, v) falls in
        c = int(x)        # col index of the cell that (u, v) falls in

        n_closest = MaxHeap()

        # first examine the row containing center cell
        for j in range(c-2, c+3):
            d = self.dist((x, y), r, c, r, j)
            if n_closest.size() < n:
                n_closest.insert(d)
            elif d >= n_closest.peek():
                continue
            else:
                n_closest.pop()
                n_closest.insert(d)

        # then examine the row above center cell
        # skip if the min possible distance achievable is larger than heap max
        min_possible = y - r + 0.5 * (1 - self.randomness)
        if min_possible < n_closest.peek():
            for j in range(c-2, c+3):
                d = self.dist((x, y), r, c, r-1, j)
                if n_closest.size() < n:
                    n_closest.insert(d)
                elif d >= n_closest.peek():
                    continue
                else:
                    n_closest.pop()
                    n_closest.insert(d)

        # then examine the row below center cell
        # skip if the min possible distance achievable is larger than heap max
        min_possible = r + 1 - y + 0.5 * (1 - self.randomness)
        if min_possible < n_closest.peek():
            for j in range(c-2, c+3):
                d = self.dist((x, y), r, c, r+1, j)
                if n_closest.size() < n:
                    n_closest.insert(d)
                elif d >= n_closest.peek():
                    continue
                else:
                    n_closest.pop()
                    n_closest.insert(d)

        # then examine two rows above center cell
        min_possible = y - r + 0.5 * (1 - self.randomness) + 1
        if min_possible < n_closest.peek():
            for j in range(c-1, c+2):
                d = self.dist((x, y), r, c, r-2, j)
                if n_closest.size() < n:
                    n_closest.insert(d)
                elif d >= n_closest.peek():
                    continue
                else:
                    n_closest.pop()
                    n_closest.insert(d)

        # then examine two rows below center cell
        min_possible = r + 1 - y + 0.5 * (1 - self.randomness) + 1
        if min_possible < n_closest.peek():
            for j in range(c-1, c+2):
                d = self.dist((x, y), r, c, r+2, j)
                if n_closest.size() < n:
                    n_closest.insert(d)
                elif d >= n_closest.peek():
                    continue
                else:
                    n_closest.pop()
                    n_closest.insert(d)

        ret = list()
        while n_closest.size() > 0:
            ret.append(n_closest.pop())
        return ret



