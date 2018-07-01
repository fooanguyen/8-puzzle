import sys
import random
import math
from struct import pack
import heapq
 
class Puzzle:
    def __init__(self, n):
	#initialize table size and range
        self.Nrange = n
        self.tableSize = n * n
 
	#set goal
        self.GOAL = range(1, self.tableSize)
        self.GOAL.append(0)

        # queue
        self.queue = []
        self.enqueued = {}

        # h
        self.h = self.heuristics

	
	#set up a slider
	self.slider = {}
	for i in range(self.tableSize):
	    sarray = []
	    if (i % self.Nrange) - 1 >= 0:
		sarray.append(i - 1)

	    if i + self.Nrange < self.tableSize:
		sarray.append(i + self.Nrange)

	    if i - self.Nrange >= 0:
		sarray.append(i - self.Nrange)
	    
	    if (i % self.Nrange) + 1 < self.Nrange:
		sarray.append(i + 1)

	    self.slider[i] = sarray
    #print box in grid 
    def printBox(self, box):
        for i in range(self.tableSize):
            if box[i]:
                print '%(n)#2d' % {'n': box[i]},
            else:
                print '  ',
            if i % self.Nrange == self.Nrange - 1:
                print

    #determine if problem has a solution
    def problemSolve(self, box):
        x = 0
        for i in range(len(box)):
            val = box[i]
            if val < 2 :
                continue
            for j in box[i:]:
                if j == 0:
                    continue
                if val > j:
                    x = x + 1
        return (x & 1) == 0


    def heuristics(self, box):
        return 0;
 
    #manhattan calculation
    def manhattanDistance(self, box):
        h = 0
        for i in range(self.tableSize):
            n = box[i]
            if n == 0:
                continue
            h += int(abs(n - 1 - i) / self.Nrange) + (abs(n - 1 - i) % self.Nrange)
        return h
 
     #swap boxes
    def swap(self, box, a, b):
        box[a], box[b] = box[b], box[a]
        return box

    #append the box after swap
    def adjacentBox(self, box):
        narray = []
        k = box.index(0)
        for i in self.slider[k]:
            narray.append(self.swap(list(box), k, i))
        return narray

    #function to call to determine problem can be solve or not
    def ableToSolve(self, initial):
        if not self.problemSolve(initial):
            return None
 
	#determine if goal is met at initial stage
        status = (initial, None, self.h(initial), 0);
        if initial == self.GOAL:
            return status
 
	#add state to heap
        self.enqueue(status)
 
        while True:
	    #pop state out of heap
            status = self.dequeue()
            if (not status):
                break
 
            (box, parent, h, g) = status
            adjacentBox = self.adjacentBox(box)
            for n_box in adjacentBox:
		#check for goal
                if n_box == self.GOAL:
                    return (n_box, status, 0, g + 1)
 
		#packing data
                packed = pack(self.tableSize*'B', *n_box)
                if (packed in self.enqueued):
                    continue;
                self.enqueued[packed] = True                       
 
                n_status = (n_box, status, self.h(n_box), g + 1)
                self.enqueue(n_status)
 
    #queuing on to the heap
    def enqueue(self, status):
        (box, parent, h, g) = status
        f = h + g;
        heapq.heappush(self.queue, (f, status))
 
    #popping out of the heap
    def dequeue(self):
        if len(self.queue) <= 0:
            return None
        (f, status) = heapq.heappop(self.queue)
        return status

def main():

    #set up arguments
    n = int(sys.argv[1])
    random.seed(sys.argv[2])

    #create a list of numbers
    initial = list(range(0,n+1))

    #shuffle the list
    random.shuffle(initial)
  
    result = Puzzle(int(math.sqrt(len(initial))))

    #display if there i a solution
    status = result.ableToSolve(initial)
    if not status:
        print "Unable to find a solution"
        return 1
 
    
    print "Initial State"
    solution = []
    while status:
        (box, parent, h, g) = status
        solution.insert(0, box)
        status = parent

    for box in solution:
        result.printBox(box)
	print "---------------"
    result.h = result.manhattanDistance	

if __name__ == '__main__':
    main()
    