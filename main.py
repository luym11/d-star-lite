import heapq
import pygame
import numpy as np
import pickle

from graph import Node, Graph
from grid import GridWorld
from utils import stateNameToCoords
from d_star_lite import initDStarLite, moveAndRescan

# Define some colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
GRAY1 = (145, 145, 102)
GRAY2 = (77, 77, 51)
BLUE = (0, 0, 80)

colors = {
    0: WHITE,
    1: GREEN,
    -1: GRAY1,
    -2: GRAY2
}

# This sets the WIDTH and HEIGHT of each grid location
WIDTH = 40
HEIGHT = 40

# This sets the margin between each cell
MARGIN = 5

# # Initialize pygame
# pygame.init()

X_DIM = 20
Y_DIM = 20
VIEWING_RANGE = 5
FIRSTTIME_VIEWING_RANGE = 20

# # Set the HEIGHT and WIDTH of the screen
# WINDOW_SIZE = [(WIDTH + MARGIN) * X_DIM + MARGIN,
#                (HEIGHT + MARGIN) * Y_DIM + MARGIN]
# screen = pygame.display.set_mode(WINDOW_SIZE)

# Set title of screen
# pygame.display.set_caption("D* Lite Path Planning")

# Loop until the user clicks the close button.
done = False

# Used to manage how fast the screen updates
# clock = pygame.time.Clock()

if __name__ == "__main__":
    # add static obstacles
    Wall = [[6,0],
            [6,1],
            [6,2],
            [6,3],
            [6,4],
            [6,5],
            [10,0],
            [10,1],
            [10,2],
            [10,3],
            [10,4],
            [10,5],
            [6,14],
            [7,14],
            [8,14],
            [9,14],
            [10,14],
            [11,14],
            [12,14],
            [13,14],
            [15,14],
            [16,14],
            [17,14],
            [18,14],
            [19,14],
            [17,9],
            [17,10],
            [17,11],
            [17,12],
            [17,13]]
    wallArray = np.array(Wall)

    # add evolving fire initial locations
    Fire = [[9,12],[3,6],[7,9],[7,10],[8,10]] # hard coded the observed fire
    initFireArray = np.array(Fire)

    monteCarloFireMap = pickle.load(open('MCFS_cur', "rb"))
    monteCarloFireMapArray = np.array(monteCarloFireMap)

    # basicfont = pygame.font.SysFont('Comic Sans MS', 36)
    
    successCounter = 0
    runCounter = 0
    for EPISODE_NUMBER in range(len(monteCarloFireMap)):
        runCounter += 1
        try:
            done = False
            graph = GridWorld(X_DIM, Y_DIM)
            s_start = 'x8y7' # actually (8,8), do this because of the moveAndRescan function
            s_goal = 'x8y16'
            goal_coords = stateNameToCoords(s_goal)

            graph.setStart(s_start)
            graph.setGoal(s_goal)
            k_m = 0
            s_last = s_start
            queue = []

            for i in range(len(Wall)):
                graph.cells[wallArray[i,1]][wallArray[i,0]] = -1
            for i in range(len(Fire)):
                graph.cells[initFireArray[i][1]][initFireArray[i][0]] = -1

            graph, queue, k_m = initDStarLite(graph, queue, s_start, s_goal, k_m)
            s_current = s_start
            pos_coords = stateNameToCoords(s_current)
            spaceCounter = 0
            firstTimeScanFlag = 1
            # -------- Main Program Loop -----------
            while not done:
                if firstTimeScanFlag == 1:
                    fireArray = initFireArray
                else:
                    fireArray = np.nonzero(monteCarloFireMapArray[EPISODE_NUMBER][spaceCounter])
                    spaceCounter += 1
                if firstTimeScanFlag == 1:
                    for i in range(len(fireArray)):
                        try:
                            graph.cells[fireArray[i][1]][fireArray[i][0]] = -1
                        except:
                            print("coordinates are [{0}, {1}]".format(fireArray[i][1],fireArray[i][0]))
                            print("length of fireArray is {0}".format(len(fireArray)))
                else:
                    for i in range(len(fireArray[0])):
                        try:
                            graph.cells[fireArray[1][i]][fireArray[0][i]] = -1
                        except:
                            print("coordinates are [{0}, {1}]".format(fireArray[i][1],fireArray[i][0]))
                            print("length of fireArray is {0}".format(len(fireArray)))
                if firstTimeScanFlag == 1:
                    s_new, k_m = moveAndRescan(
                        graph, queue, s_current, FIRSTTIME_VIEWING_RANGE, k_m)
                    firstTimeScanFlag = 0
                else:
                    s_new, k_m = moveAndRescan(
                        graph, queue, s_current, VIEWING_RANGE, k_m)
                if s_new == 'goal':
                    print('Goal Reached!')
                    done = True
                    successCounter += 1
                else:
                    # print('setting s_current to ', s_new)
                    s_current = s_new
                    pos_coords = stateNameToCoords(s_current)
                    # print('got pos coords: ', pos_coords)
        except:
            pass
    print(successCounter/len(monteCarloFireMap))
    print(runCounter)