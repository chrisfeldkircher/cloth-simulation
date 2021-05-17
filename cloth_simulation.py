import pygame
import random
from time import time 
import numpy as np
from itertools import combinations
import operator

class Particle:
    def __init__(self, pos):
        self.pos = pos
        self.p_pos = pos
        self.mass = 100.0
        self.gravity = -self.mass*9.81#-1100
        self.pinned = False
        self.dragged = False
        self.dim = [1000, 600]
        self.a_y = 0.0
        self.a_x = 0.0
        self.springs = []

    def update(self, dt):
        if (not self.pinned):
            x = round(self.pos[1], 5)
            y = round(self.pos[0], 5)

            p_x = round(self.p_pos[1], 5)
            p_y = round(self.p_pos[0], 5)

            self.applyForce(0, self.gravity)

            v_y = p_y - y
            v_y = round(v_y * 0.99, 5)

            v_x = p_x - x
            v_x = round(v_x * 0.99, 5)

            #dt_squared = dt **2

            n_y = y - v_y - 0.5 * self.a_y * dt **2
            n_x = x - v_x - 0.5 * self.a_x * dt **2

            self.p_pos = list(self.pos)

            self.pos[1] = round(n_x, 5)
            self.pos[0] = round(n_y, 5)

            self.a_x = 0
            self.a_y = 0

    def const(self):
        if (not self.pinned):
            if (self.pos[0] < 50):
                self.pos[0] = 50

            if (self.pos[0] > self.dim[0]):
                self.pos[0] = self.dim[0]

            if (self.pos[1] < 50):
                self.pos[1] = 50

            if (self.pos[1] > self.dim[0]):
                self.pos[1] = self.dim[0]

        for spring in self.springs:
            spring.const()

    def applyForce(self, x, y):
        self.a_x = self.a_x + x
        self.a_y = self.a_y + y

class Spring:
    def __init__(self, p1, p2, restingDist):
        self.point1 = p1
        self.point2 = p2
        self.rest = restingDist
        #self.stiffness = 0.1 #elastic
        #self.stiffness = 1 #rigid
        self.stiffness = 0.7 #rigid
        
    def const(self):
        v_x = self.point1.pos[1] - self.point2.pos[1]
        v_y = self.point1.pos[0] - self.point2.pos[0]
        hyp = np.sqrt(v_x**2 + v_y**2)
        hyp = 1 if hyp == 0 else hyp
		
        diff = (self.rest - hyp) / hyp		

        trans_x = v_x * 0.5 * diff * self.stiffness
        trans_y = v_y * 0.5 * diff * self.stiffness		

        if (not self.point1.pinned):
            self.point1.pos[1] = self.point1.pos[1] + trans_x
            self.point1.pos[0] = self.point1.pos[0] + trans_y

        if (not self.point2.pinned):
            self.point2.pos[1] = self.point2.pos[1] - trans_x
            self.point2.pos[0] = self.point2.pos[0] - trans_y

class Simulation:
    def __init__(self):
        self.sq_size = 20
        self.width = 20
        self.height = 20
        self.pnts = []
        self.pnt_drag = False
        self.pointDragged = [0, 0]
        self.pinned_pnts = []
        self.points = self.createNet(self.sq_size, self.width, self.height, 150, 150, self.pinned_pnts)
        self.dt = 0.016
        self.accuracy = 4
        self.dim = 1000, 600
        self.color = []
        self.tear_distance = 60
        self.wind = False
        self.gravity = False
            
    def init(self):
        pygame.init()
        pygame.display.set_caption("Cloth Simulation")
        self.screen = pygame.display.set_mode(self.dim)
        self.isRunning = True
        self.screen.fill((255,255,255))
        self.clock = pygame.time.Clock()
        self.color_generator()

    def apply_force(self):
        pointIndex = 0
        pointsList = []
        x, y = pygame.mouse.get_pos()

        for point in self.points:
            pointsList.append([round(point.pos[0]), round(point.pos[1])])

        for extraX in range(-15, 15):
            for extraY in range(-15, 15):
                try:
                    pointIndex = pointsList.index([y + extraY, x + extraX])
                except ValueError:
                    pass
                else:
                    if (not self.points[pointIndex].pinned):
                        self.pnt_drag = True
                        self.pointDragged = pointIndex
                    else:
                        self.points[pointIndex].pinned = False
    
    def wind_force(self, p1, p2, direction):
        vec1 = [p2.pos[0] - p1.pos[0], p2.pos[1] - p1.pos[1]]
        norm = [vec1[1], -vec1[0]]
        unit_n = np.array((norm[0]/np.linalg.norm(norm), norm[1]/np.linalg.norm(norm)))
        force = [norm[0]*np.dot(unit_n, direction), norm[1]*np.dot(unit_n, direction)]
        p1.applyForce(force[0], force[1])
        p1.applyForce(force[0], force[1])

    def createNet(self, sq_size, width, height, posX, posY, pinned):
        self.pnts = [Particle([300, 0]) for x in range(width*height)] #y=[0], x=[1]

        for point in range(0, len(self.pnts)):
			# Link to point to the right (rows)
            if ((point+1) <= len(self.pnts) and (point+1) % width != 0):
                self.pnts[point].springs.append(Spring(self.pnts[point], self.pnts[(point+1)], sq_size))

			# Link to point above (columns)
            if ((point+width) <= len(self.pnts)-1):
                self.pnts[point].springs.append(Spring(self.pnts[point], self.pnts[point+width], sq_size))

		# Pin the top two pnts
        """
        pnts[0].pinned = True
        pnts[0].pos = [50, 250]
        pnts[width-1].pinned = True
        pnts[width-1].pos = [50, 650]
        """
        for i in range(0, width):
            self.pnts[i].pinned = True
            self.pnts[i].pos = [50,250+400/20*i]
            pinned.append(self.pnts[i])

        return self.pnts
    
    def tear(self):
        x, y = pygame.mouse.get_pos()
        self.points[self.pointDragged].pos = [y, x]
        distance = list(map(operator.sub, self.points[self.pointDragged].pos, self.points[self.pointDragged + 1].pos))
        if np.hypot(*distance) > self.tear_distance:
            for i in range(len(self.points[self.pointDragged].springs)):
                self.points[self.pointDragged].springs.clear()
            for i in range(len(self.points[self.pointDragged + 1].springs)):
                self.points[self.pointDragged + 1 ].springs.clear()

    def event(self, event):
        if event.type == pygame.QUIT:
            self.isRunning = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                self.apply_force()
        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                self.pnt_drag = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                self.wind = not self.wind
            if event.key == pygame.K_SPACE:
                self.gravity = not self.gravity

    def loop(self):
        for accuracy in range(0, self.accuracy):
            for point in self.points:
                point.const()

        for point in self.points:
            point.update(self.dt)

        if (self.pnt_drag):
            self.tear()

        if self.wind:
            for i in range(len(self.points) -1):
                self.wind_force(self.points[i], self.points[i+1], (random.randint(-45, 30), random.randint(-50, 70)))
        if self.gravity:
            for points in self.points:
                points.gravity = 0
        else:
            for points in self.points:
                points.gravity = -points.mass*9.81

    def color_generator(self):
        for i in range(self.width):
            r = random.randint(50,255)
            g = random.randint(50,255)
            b = random.randint(50,255)
            self.color.append((r,g,b))

    def draw(self):
        self.screen.fill((255,255,255))
        i = -1
        for point in self.points:
            pygame.draw.circle(self.screen, (100,0,0), (point.pos[1], point.pos[0]), 1)
            for spring in point.springs:
                i %= 5
                p1 = spring.point1
                p2 = spring.point2
                pygame.draw.line(self.screen, self.color[i], (int(round(p1.pos[1])), int(round(p1.pos[0]))), (int(round(p2.pos[1])), int(round(p2.pos[0]))), 1)
            i += 1

        pygame.display.flip()

    def execute(self):
        if self.init() == False:
            self.isRunning = False

        while self.isRunning:
            for event in pygame.event.get():
                self.event(event)
            
            #self.clock.tick(60)
            self.loop()
            self.draw()

        pygame.quit()

if __name__ == "__main__":
    p = Simulation()
    p.execute()
