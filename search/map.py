import copy
import math
from search.algorithms import State
import numpy as np
import random

class Map:
    """
    Class to store the map. The maps in folder dao-map are from movingai.org.
    """
    def __init__(self, file_name):
        self.file_name = file_name        
        self.map_file = open(self.file_name)
        self.type_map = self.map_file.readline()
        self.height = int(self.map_file.readline().split(' ')[1])
        self.width = int(self.map_file.readline().split(' ')[1])
        
        State.map_width = self.width
        State.map_height = self.height
        
        self.read_map()
        self.convert_data()
        
        self.map_file.close()
        
    def read_map(self):
        """
        Reads map from the file and stores it in memory.
        """
        line = self.map_file.readline()
        while 'map' not in line:
            line = self.map_file.readline()
        lines = self.map_file.readlines()

        self.data_str = []
        for line in lines:
            line_list = []
            line = line.replace('\n', '')
            for c in line:
                line_list.append(c)
            self.data_str.append(line_list)
        
    def convert_data(self):
        """
        Converts the map, initially in the movingai.org format, to a matrix of integers, where
        traversable cells have the value of 1 and non-traversable cells have the value of 0.
        
        The movingai.com maps are encoded as follows. 
        
        . - passable terrain
        G - passable terrain
        @ - out of bounds
        O - out of bounds
        T - trees (unpassable)
        S - swamp (passable from regular terrain)
        W - water (traversable, but not passable from terrain)
        """
        self.data_int = np.zeros((len(self.data_str), len(self.data_str[0])))

        for i in range(0, self.height):
            for j in range(0, self.width):
                if self.data_str[i][j] == '.' or self.data_str[i][j] == 'G':
                    self.data_int[i][j] = 0
                else:
                    self.data_int[i][j] = 1        
    
    def plot_map(self, closed_data, start, goal, filename):
        import matplotlib.pyplot as plt

        data_plot = copy.deepcopy(self.data_int)
        data_plot *= 100

        for i in range(0, self.height):
            for j in range(0, self.width):
                if data_plot[i][j] == 0:
                    data_plot[i][j] = -100

        for _, state in closed_data.items():
            data_plot[state.get_y()][state.get_x()] = 1

        data_plot[start.get_y()][start.get_x()] = -50
        data_plot[goal.get_y()][goal.get_x()] = -50

        plt.axis('off')
        plt.imshow(data_plot, cmap='Greys', interpolation='nearest')
        plt.savefig(filename)
        # plt.show()

    def plot_map_list(self, points, filename):
        import matplotlib.pyplot as plt

        data_plot = copy.deepcopy(self.data_int)
        data_plot *= 100

        for i in range(0, self.height):
            for j in range(0, self.width):
                if data_plot[i][j] == 0:
                    data_plot[i][j] = -100

        for state in points:
            data_plot[state.get_y()][state.get_x()] = 1

        plt.axis('off')
        plt.imshow(data_plot, cmap='Greys', interpolation='nearest')
        plt.savefig(filename)
        # plt.show()

    def random_state(self):
        """
        Generates a valid random state for a given map. 
        """
        x = random.randint(0, self.width - 1)
        y = random.randint(0, self.height - 1)
        while self.data_int[y][x] == 1:
            x = random.randint(0, self.width - 1)
            y = random.randint(0, self.height - 1)
        state = State(x, y)
        return state
    
    def is_valid_pair(self, x, y):
        """
        Verifies if an x-y pair is valid for a map.
        """
        if x < 0 or y < 0:
            return False
        if x >= self.width or y >= self.height:
            return False
        if self.data_int[y][x] == 1:
            return False
        return True
    
    def cost(self, x, y):
        """
        Returns the cost of an action.
        
        Diagonal moves cost 1.5; each action in the 4 cardinal directions costs 1.0
        """
        if x == 0 or y == 0:
            return 1
        else:
            return 1.5
    
    def successors(self, state, constraints=None):
        """
        Transition function: receives a state and returns a list with the neighbors of that state in the space
        """
        children = []
        for i in range(-1, 2):
            for j in range(-1, 2):

                if i == 0 or j == 0:
                    if self.is_valid_pair(state.get_x() + i, state.get_y() + j):
                        if constraints is None or (state.get_x() + i, state.get_y() + j) not in constraints:
                            s = State(state.get_x() + i, state.get_y() + j)
                            s.set_g(state.get_g() + 1)
                            children.append(s)
                        elif (state.get_g() + 1) not in constraints[(state.get_x() + i, state.get_y() + j)]:
                            s = State(state.get_x() + i, state.get_y() + j)
                            s.set_g(state.get_g() + 1)
                            children.append(s)
        return children