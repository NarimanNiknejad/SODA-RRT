import random
import math
from sre_constants import SUCCESS
import pygame 
import logging
import csv
import cvxpy as cp
import numpy as np
import scipy.linalg as linalg
# import control
import random
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import math
import scipy.stats as stats
import numpy as np

import seaborn as sns


class RRTMap:
    def __init__(self,start, goal, MapDimension,Real_MapDimension, obsdim, obsnum,MapwindowName="SODA-RRT path planning",goal_radius=0.45) -> None:
        self.start = start
        self.goal = goal
        self.goal_radius = goal_radius


        self.MapDimension = MapDimension
        self.Maph,self.Mapw = self.MapDimension

        # window settings
        self.MapwindowName = MapwindowName
        pygame.display.set_caption(self.MapwindowName)
        self.map = pygame.display.set_mode((self.Mapw,self.Maph))
        self.map.fill((255,255,255))
        self.nodeRad = 2
        self.nodeThickness = 0
        self.edgeThickness = 2

        self.obstacles = []
        self.obsdim = obsdim
        self.obsNumber = obsnum

        self.min_x = -Real_MapDimension[0]
        self.max_x = Real_MapDimension[0]
        self.min_y = -Real_MapDimension[1]
        self.max_y = Real_MapDimension[1]

        #colors
        self.Grey = (170,170,170)
        self.BOUNDARY_COLOR = (150, 190, 30)  
        self.Blue = (70,70,250)
        self.Green = (0,250,0)
        self.Red = (250,0,0)
        self.elColor = (21, 171, 0)
        # Set the color palette and generate 100 distinct colors
        sns.set_palette("husl")
        distinct_colors = sns.color_palette(n_colors=100)

        # Convert the distinct colors to RGB format
        self.colors = [(int(r * 255), int(g * 255), int(b * 255)) for r, g, b in distinct_colors]


    def map_to_screen(self,x_meters, y_meters, screen_width = 800, screen_height = 800):

        # Calculate the scale factors for conversion
        x_scale = screen_width / (self.max_x - self.min_x)
        y_scale = screen_height / (self.max_y - self.min_y)
        
        x_pixels = (x_meters - self.min_x) * x_scale
        y_pixels = (-y_meters + self.max_y) * y_scale
        return int(x_pixels), int(y_pixels)

    def map_to_reality(self,x_pixels, y_pixels, screen_width = 800, screen_height = 800):

        # Calculate the scale factors for conversion
        x_scale = screen_width / (self.max_x - self.min_x)
        y_scale = screen_height / (self.max_y - self.min_y)
        
        x_meters = x_pixels / x_scale + self.min_x
        y_meters = -(y_pixels/y_scale -self.max_y)
        
        return x_meters, y_meters

    def drawMap(self,obstacles):

        self.start_in_pixels = self.map_to_screen(self.start[0],self.start[1])
        self.goal_in_pixels = self.map_to_screen(self.goal[0],self.goal[1])

        goal_radius_pixels = self.goal_radius/(2*self.max_x)*self.Mapw

        pygame.draw.circle(self.map,self.Green,self.start_in_pixels,self.nodeRad+5.0)
        pygame.draw.circle(self.map,self.Red,self.goal_in_pixels,goal_radius_pixels,3)
        self.drawObs(obstacles)
       
    def drawPath(self,path,Ps):

        for i,node in enumerate(path):
     
            pygame.draw.circle(self.map, self.Red,node, self.nodeRad+3,0)
            if len(Ps)> 0:
                self.drawEllip(node[0],node[1],Ps[i],l=i)

    def drawObs(self,obstacles):
        obstaclesList = obstacles.copy()
        while(len(obstaclesList)>0):
            obstacle = obstaclesList.pop(0)
            pygame.draw.rect(self.map,self.BOUNDARY_COLOR,obstacle)

    def drawEllip(self,x_c,y_c,Pss,l):

        # print(x_c,y_c,Pss)

        x_c, y_c = self.map_to_reality(x_c,y_c)

        # Define the range for x and y
        x_range = np.linspace(self.min_x, self.max_x, self.Mapw)
        y_range = np.linspace(self.min_y, self.max_y, self.Maph)

        # Create a grid of points
        x, y = np.meshgrid(x_range, y_range)

        P = np.array(Pss)
        # Calculate the ellipsoid equation for each point
        z = (x-x_c)**2 * P[0,0] + (y-y_c)**2 * P[1,1] + 2 * (x-x_c) * (y-y_c) * P[0,1]

        # Draw the contour lines of the ellipsoid
        for level in [1.0]:
            for i in range(self.Mapw):
                for j in range(self.Maph):
                    if abs(z[j, i] - level) < 0.01:
                        x_pixel, y_pixel = self.map_to_screen(x[j, i], y[j, i])
                        pygame.draw.circle(self.map, self.colors[l], (x_pixel, y_pixel), 1)




class RRTGraph:

    def __init__(self,start, goal, MapDimension,Real_MapDimension, obsdim, obsnum, A, B, W, Q, R, lamda,goal_radius=0.45) -> None:
        
        # Goal radius

        self.goal_radius = goal_radius

        # Dynamics part
        self.A, self.B  = A, B
        self.W, self.Q, self.R, self.lamda = W, Q, R, lamda

        # print(self.B)

        # Graph part
        P_start = [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]


        (x,y) = start
        self.start = start
        self.goal = goal

        self.min_x = -Real_MapDimension[0]
        self.max_x = Real_MapDimension[0]
        self.min_y = -Real_MapDimension[1]
        self.max_y = Real_MapDimension[1]

        self.start_in_pixels = self.map_to_screen(self.start[0],self.start[1])
        self.goal_in_pixels = self.map_to_screen(self.goal[0],self.goal[1])

        self.goalFlag = False
        self.Maph, self.Mapw = MapDimension[0], MapDimension[1]
        self.Maph_R, self.Mapw_R = Real_MapDimension[0],Real_MapDimension[1]
        self.Ps = []
        self.Ks = []
        self.borders = []
        self.x = []
        self.y = []
        self.x_in_pixels = []
        self.y_in_pixels = []
        self.parent = []
        


        #initialize the tree

        self.x.append(x)
        self.y.append(y)

        self.x_in_pixels.append(self.map_to_screen(x,y)[0])
        self.y_in_pixels.append(self.map_to_screen(x,y)[1])

        self.Ps.append(P_start)
        self.Ks.append(np.array([[0,0],[0,0]]))
        self.parent.append(0)
        self.borders.append([0,0,0,0])

        #the obstacles
        self.obstacles_in_pixels = []
        self.obsDim = obsdim
        self.obsDim_in_pixels = int(self.obsDim/(2*self.max_x)*self.Mapw)
        self.obsNum = obsnum
        #path
        self.goalstate = None
        self.path = []




    def lamda_contractive_LQR_control_model_based(self,A, B, W, Q, R, M, lamda):
    
        # State and input dimensions
        n, m = B.shape
        I = np.eye(n)

        Ps = cp.Variable((n, n), symmetric=True)
        Po = cp.Variable((n, n), symmetric=True)
        H = cp.Variable((n, n), symmetric=True)
        L = cp.Variable((m, m))
        F = cp.Variable((m, n))
        psi = cp.Variable((1, 1))
        eps = cp.Variable((1, 1))
        gamma = cp.Variable((1, 1))

        obj = 0.0001*cp.log_det(H) + gamma - cp.log_det(Ps)
        
        # cons = [Ps - Psafe >> 0]
        
        cons = [M[0]@Ps@M[0].T <= 1]
        cons += [M[1]@Ps@M[1].T <= 1]
        cons += [M[2]@Ps@M[2].T <= 1]
        cons += [M[3]@Ps@M[3].T <= 1]

        cons += [M[4]@Ps@M[4].T <= 1]
        cons += [M[5]@Ps@M[5].T <= 1]
        cons += [M[6]@Ps@M[6].T <= 1]
        cons += [M[7]@Ps@M[7].T <= 1]


        cons += [Po >> I]
        cons += [H >> 0]
        cons += [
            cp.bmat([[Po-I, (A@Ps+B@F)], [(A@Ps+B@F).T, H]]) >> 0
        ]
        cons += [
            cp.bmat([[lamda*Ps, (A@Ps+B@F).T], [A@Ps+B@F, Ps]]) >> 0
        ]
        cons += [
            cp.bmat([[L, F], [F.T, H]]) >> 0
        ]
        cons += [
            cp.bmat([[H, Ps], [Ps, Po]]) >> 0
        ]
        # cons += [ cp.lambda_max(Ps) <= psi]
        

        # cons += [ cp.lambda_max(H) <= eps]
        cons += [ cp.trace(Q@Po)+cp.trace(R@L) <= gamma]
        cons += [gamma >= 0]
        cons += [eps >= 0]
        cons += [psi >= 0]

        prob = cp.Problem(cp.Minimize(obj), cons)
        prob.solve(solver=cp.MOSEK, verbose=False)
        K = F.value@np.linalg.inv(Ps.value)
        # print(K)
        
        return K, np.linalg.inv(Ps.value), Po.value, (gamma.value, psi.value, prob.value)    

    def map_to_screen(self,x_meters, y_meters, screen_width = 800, screen_height = 800):

        # Calculate the scale factors for conversion
        x_scale = screen_width / (self.max_x - self.min_x)
        y_scale = screen_height / (self.max_y - self.min_y)
        
        x_pixels = (x_meters - self.min_x) * x_scale
        y_pixels = (-y_meters + self.max_y) * y_scale
        return int(x_pixels), int(y_pixels)


    def map_to_reality(self,x_pixels, y_pixels, screen_width = 800, screen_height = 800):

        # Calculate the scale factors for conversion
        x_scale = screen_width / (self.max_x - self.min_x)
        y_scale = screen_height / (self.max_y - self.min_y)
        
        x_meters = x_pixels / x_scale + self.min_x
        y_meters = -(y_pixels/y_scale -self.max_y)
        
        return x_meters, y_meters

       
    # def makeRandomRect(self):

    #     #this is to create random obstacle location
    #     uppercornerx = int(random.uniform(0,self.Mapw-self.obsDim))
    #     uppercornery = int(random.uniform(0,self.Maph-self.obsDim))

    #     return (uppercornerx,uppercornery)


    def makeRandomRect_fromMeters(self):
        #this is to create random obstacle location
        uppercornerx = (random.uniform(-self.Mapw_R,self.Mapw_R-self.obsDim))
        uppercornery = (random.uniform(-self.Mapw_R,self.Maph_R-self.obsDim))

        return (uppercornerx,uppercornery)

    def makeCenterRect_fromMeters(self):
        #this is to create center obstacle location
        uppercornerx = -self.obsDim/2
        uppercornery = self.obsDim/2

        return (uppercornerx,uppercornery)

    def makeObs_fromMeters(self):

        obs_in_pixels = []
        obscoord = []
        obscoord_in_pixels = []

        for i in range(0,self.obsNum):
            rectang = None
            startgoalcol = True 
            while startgoalcol:
                upper = self.makeRandomRect_fromMeters()
                x_up,y_up  = self.map_to_screen(upper[0],upper[1])
                x_up = int(x_up)
                y_up = int(y_up)
                obsDim_in_pixels = int(self.obsDim/(2*self.max_x)*self.Mapw)
                rectang=pygame.Rect((x_up,y_up),(obsDim_in_pixels ,obsDim_in_pixels))
                rectangcoord = upper
                # rectangcoord_in_pixels = upper_in_pixels
                if rectang.collidepoint(self.start_in_pixels) or rectang.collidepoint(self.goal_in_pixels):
                    startgoalcol = True 
                else:
                    startgoalcol = False

            obs_in_pixels.append(rectang)

            obscoord_in_pixels.append([x_up,y_up,obsDim_in_pixels])
            obscoord.append([rectangcoord[0],rectangcoord[1],self.obsDim])

        self.obstacles_in_pixels = obs_in_pixels.copy() #this one is in pixels
        self.obstaclescoord = obscoord.copy()
        self.obstaclescoord_in_pixels = obscoord_in_pixels.copy()

        # in obstaclescooord we save the x, y, and also the dimension of the obstacles as [x,y,dim]
        # print(obs_in_pixels)

        return obs_in_pixels

    def makeObs_fromMeters_at_center(self):

        obs_in_pixels = []
        obscoord = []
        obscoord_in_pixels = []

        rectang = None
        startgoalcol = True 


        upper = self.makeCenterRect_fromMeters()
        x_up,y_up  = self.map_to_screen(upper[0],upper[1])
        x_up = int(x_up)
        y_up = int(y_up)
        obsDim_in_pixels = int(self.obsDim/(2*self.max_x)*self.Mapw)
        rectang=pygame.Rect((x_up,y_up),(obsDim_in_pixels ,obsDim_in_pixels),width=1)
        rectangcoord = upper


        obs_in_pixels.append(rectang)

        obscoord_in_pixels.append([x_up,y_up,obsDim_in_pixels])
        obscoord.append([rectangcoord[0],rectangcoord[1],self.obsDim])

        self.obstacles_in_pixels = obs_in_pixels.copy() #this one is in pixels
        self.obstaclescoord = obscoord.copy()
        self.obstaclescoord_in_pixels = obscoord_in_pixels.copy()

        # in obstaclescooord we save the x, y, and also the dimension of the obstacles as [x,y,dim]
        # print(obs_in_pixels)

        return obs_in_pixels

    def add_node(self,n,x,y):
        #n is id of the node
        #x and y are the coordiantes of the node
        self.x.insert(n,x)
        self.y.insert(n,y)

        x_pix, y_pix = self.map_to_screen(x,y)
        self.x_in_pixels.insert(n,x_pix)
        self.y_in_pixels.insert(n,y_pix)

    def remove_node(self,n):

        self.x.pop(n)
        self.y.pop(n)
        self.x_in_pixels.pop(n)
        self.y_in_pixels.pop(n)


    def add_edge(self,parent,child):
        #in this method, parent is the element and the index is the child: meaning that for example the 4th element if it is 2, the child which is
        #the 4th node is connected to the 2nd node as its parent
        self.parent.insert(child,parent)
       
    def remove_edge(self,n):
        self.parent.pop(n)

    def number_of_nodes(self):
        return (len(self.x))

    def distance(self,n1,n2):
        (x1,y1) = (self.x[n1],self.y[n1])
        (x2,y2) = (self.x[n2],self.y[n2])
        px = (float(x1)-float(x2))**2
        py = (float(y1)-float(y2))**2

        return((px+py)**(0.5)) #Euclidean distance

    def cost_ellipsoidal(self,n,P,x_c,y_c):
        P = np.array(P)
        # Calculate the ellipsoid equation for each point
        z = (self.x[n]-x_c)**2 * P[0,0] + (self.y[n]-y_c)**2 * P[1,1] + 2 * (self.x[n]-x_c) * (self.y[n]-y_c) * P[0,1]
        return z

   
    def sample_envir_pixel(self):
        x = int(random.uniform(0,self.Mapw))
        y = int(random.uniform(0,self.Maph))

        return x,y

    def sample_envir(self):
        x = (random.uniform(self.min_x,self.max_x))
        y = (random.uniform(self.min_y,self.max_y))

        return x,y

    def sample_envir_from_goal(self):
        # Center and radius of the circle
        x_c, y_c = self.goal[0], self.goal[1]  # Example center coordinates
        rad = self.goal_radius

        # Generate random polar coordinates
        theta = 2 * math.pi * random.random()  # Random angle in radians
        r = rad * math.sqrt(random.random())  # Random radius within the circle

        # Convert polar coordinates to Cartesian coordinates
        x = x_c + r * math.cos(theta)
        y = y_c + r * math.sin(theta)

        return x,y
   
    def nearest(self,n):

        #gets the sampled node in the config sapce and checks which nodes are closest to it
        dmin = self.distance(0,n)
        nnear = 0
        for i in range(0,n):
            if (self.distance(i,n)<dmin):
                dmin = self.distance(i, n)
                nnear = i

        return nnear 


    def nearest_within_ellips(self,P,x_c,y_c):

        n = self.number_of_nodes()-1
        cost_min = 1.5
        nnear = 0

        for i in range(0,n):
            if(self.cost_ellipsoidal(i,P, x_c, y_c) < cost_min and self.cost_ellipsoidal(i,P, x_c, y_c)<=1.0):
                cost_min = self.cost_ellipsoidal(i,P,x_c,y_c)
                nnear = i
        return nnear

    def any_within_ellips(self,P,x_c,y_c):

        n = self.number_of_nodes()-1
        cost_min = 1.5

        for i in range(0,n):
            if(self.cost_ellipsoidal(i,P, x_c, y_c) < cost_min and self.cost_ellipsoidal(i,P, x_c, y_c)<=1.0):
                cost_min = self.cost_ellipsoidal(i,P,x_c,y_c)

        if(cost_min<=1.0):
            return True
        else:
            # print("removed haha!")
            self.remove_node(n)
            return False
       

    def isFree(self):
        n = self.number_of_nodes()-1
        (x_pix,y_pix) = (self.x_in_pixels[n],self.y_in_pixels[n])
        (x,y) = (self.x[n],self.y[n])

        obs = self.obstacles_in_pixels.copy()
        while len(obs):
            rectang = obs.pop(0)
            if(rectang.collidepoint(x_pix,y_pix)):
                self.remove_node(n)
                return False

        # ellips = self.Ps.copy()
        # ellips.pop(0)

        # for i, e in enumerate(ellips):
        #     if(self.cost_ellipsoidal(n,e,self.x[i],self.y[i])<1.0):
        #         self.remove_node(n)
        #         return False

        return True

    def crossObstacle(self,x1,x2,y1,y2):
        #this checks if the edge between twopoints collide with any of the obstacles
        #this is done by interpolating between the two nodes and checking if the found points collide with the obstacle

        obs = self.obstacles_in_pixels.copy()
        while (len(obs)>0):
            rectang = obs.pop(0)
            for i in range(1,101):
                u = i/100
                x = x1*u +x2*(1-u)
                y = y1*u +y2*(1-u)
                if rectang.collidepoint(x,y):
                    return True
        return False

    def find_border_Center(self,x,y,step=1):

        ch = random.uniform(0, 1)

        if(x<= -self.obsDim/2 and y>= self.obsDim/2 and ch>=0.5):
            x_l = self.min_x
            x_r = self.max_x
            y_u = self.max_y
            y_d = self.obsDim/2

        if(x<= -self.obsDim/2 and y>= self.obsDim/2 and ch<0.5):
            x_l = self.min_x
            x_r = -self.obsDim/2
            y_u = self.max_y
            y_d = self.min_y
        
        if(x>= -self.obsDim/2 and x<= self.obsDim/2 and y>= self.obsDim/2 ):
            x_l = self.min_x
            x_r = self.max_x
            y_u = self.max_y
            y_d = self.obsDim/2

        if(x>= self.obsDim/2  and y>= self.obsDim/2 and ch>=0.5):
            x_l = self.min_x
            x_r = self.max_x
            y_u = self.max_y
            y_d = self.obsDim/2
        
        if(x>= self.obsDim/2  and y>= self.obsDim/2 and ch<0.5):
            x_l = self.obsDim/2
            x_r = self.max_x
            y_u = self.max_y
            y_d = self.min_y
        
        if(x>= self.obsDim/2  and y<= self.obsDim/2 and y>= -self.obsDim/2):
            x_l = self.obsDim/2
            x_r = self.max_x
            y_u = self.max_y
            y_d = self.min_y

        if(x>= self.obsDim/2  and y<= -self.obsDim/2 and ch>=0.5):
            x_l = self.obsDim/2
            x_r = self.max_x
            y_u = self.max_y
            y_d = self.min_y
        
        if(x>= self.obsDim/2  and y<= -self.obsDim/2 and ch<0.5):
            x_l = self.min_x
            x_r = self.max_x
            y_u = -self.obsDim/2
            y_d = self.min_y

        if(x<= self.obsDim/2 and x>= -self.obsDim/2 and y<= -self.obsDim/2):
            x_l = self.min_x
            x_r = self.max_x
            y_u = -self.obsDim/2
            y_d = self.min_y

        if(x<= -self.obsDim/2 and y<= -self.obsDim/2 and ch>=0.5):
            x_l = self.min_x
            x_r = self.max_x
            y_u = -self.obsDim/2
            y_d = self.min_y

        if(x<= -self.obsDim/2 and y<= -self.obsDim/2 and ch<0.5):
            x_l = self.min_x
            x_r = -self.obsDim/2
            y_u = self.max_y
            y_d = self.min_y

        if(x<= -self.obsDim/2 and y>= -self.obsDim/2 and y<= self.obsDim/2):
            x_l = self.min_x
            x_r = -self.obsDim/2
            y_u = self.max_y
            y_d = self.min_y

        return x_r,x_l,y_u, y_d

        
        


    def find_border(self,x,y,step=1):

        # this method increamentally increases x and y of the found node to find the obstacles around it, similar to LIDAR and proximity sensors
        x,y = self.map_to_screen(x,y)
        #going to right
        right_check = 0
        x_r_stored = x
        for x_r in range (x,self.Mapw,step):
            obs = self.obstacles_in_pixels.copy()
            obsc = self.obstaclescoord_in_pixels.copy()
            while (len(obs)>0 and right_check == 0):

                rectang = obs.pop(0)
                rectangcoord = obsc.pop(0)

                for y_c in range(0,self.Maph,step):
                    if(rectang.collidepoint(x_r,y_c)):
                        right_check = 1
                        x_r_stored = rectangcoord[0]

        if(x_r == self.Mapw-1 and right_check == 0):
            x_r_stored = x_r
            right_check = 1


        #going to left
        left_check = 0
        x_l_stored = x
        for x_l in range (x,0,-step):
            obs = self.obstacles_in_pixels.copy()
            obsc = self.obstaclescoord_in_pixels.copy()
            while (len(obs)>0 and left_check == 0):
                rectang = obs.pop(0)
                rectangcoord = obsc.pop(0)
                for y_c in range(0,self.Maph,step):

                    if(rectang.collidepoint(x_l,y_c)):
                        left_check = 1
                        x_l_stored = rectangcoord[0]+ self.obsDim_in_pixels

        if(x_l == 1 and left_check == 0):
            x_l_stored = x_l
            left_check = 1

        #going upwards
        up_check = 0
        y_u_stored = y
        for y_u in range (y,0,-step):
            obs = self.obstacles_in_pixels.copy()
            obsc = self. obstaclescoord_in_pixels.copy()
            while (len(obs)>0 and up_check == 0):
                rectang = obs.pop(0)
                rectangcoord = obsc.pop(0)

                for x_c in range(0,self.Mapw,step):

                    if(rectang.collidepoint(x_c,y_u)):
                        up_check = 1
                        y_u_stored = rectangcoord[1] + self.obsDim_in_pixels
        # print(y_u)
        if(y_u == 1 and up_check == 0):
            y_u_stored = y_u
            up_check = 1

        #going downwards
        down_check = 0
        y_d_stored = y
        for y_d in range (y,self.Maph,step):
            obs = self.obstacles_in_pixels.copy()
            obsc = self. obstaclescoord_in_pixels.copy()
            while (len(obs)>0 and down_check == 0):
                rectang = obs.pop(0)
                rectangcoord = obsc.pop(0)
                for x_c in range(0,self.Mapw,step):

                    if(rectang.collidepoint(x_c,y_d)):
                        down_check = 1
                        y_d_stored = rectangcoord[1]

        if(y_d == self.Maph -1 and down_check == 0):
            y_d_stored = y_d
            down_check = 1

        # print("x_r_stored, y_u_stored, x_l_stored, y_d_stored")
        # print(x_r_stored, y_u_stored, x_l_stored, y_d_stored)
        x_r_stored_R, y_u_stored_R = self.map_to_reality(x_r_stored, y_u_stored)
        x_l_stored_R, y_d_stored_R = self.map_to_reality(x_l_stored, y_d_stored)
        return x_r_stored_R,x_l_stored_R,y_u_stored_R, y_d_stored_R
   
    def ellipsoid_gen(self, x, y):
        max_speed = 40
        #this method recieves a candidate node's coordiantes and tries to find the largest invariant set around it + the control gain related to it
        # x_r, x_l, y_u, y_d = self.find_border(x,y)
        x_r, x_l, y_u, y_d = self.find_border_Center(x,y)

        # border = [x_r, x_l, y_u, y_d]
        # x_rl = min(abs(x_r-x),abs(x_l-x))
        # y_ud = min(abs(y_u-y),abs(y_d-y))

        # print( "x_r, y_u, x_l, y_d:")
        # print( x_r, y_u, x_l, y_d)

        # border = [x_r, x_l, y_u, y_d]
        # print(border)
        x_r = x_r - x
        x_l = x_l - x
        y_u = y_u - y
        y_d = y_d - y

        # print([x_r, x_l, y_u, y_d])
        border = [x_r, x_l, y_u, y_d]
        
        # border = [x_rl, -x_rl, y_ud, -y_ud]
        # print(border)

        self.M = np.array([
            [1/x_r, 0,0,0],
            [1/x_l, 0,0,0],
            [0, 1/y_u,0,0],
            [0, 1/y_d,0,0],
            [0, 0,-1/max_speed,0],
            [0, 0,1/max_speed,0],
            [0, 0,0,-1/max_speed],
            [0, 0,0,1/max_speed]
        ])

        # self.M = np.array([
        #     [1/x_rl, 0],
        #     [-1/x_rl, 0],
        #     [0, 1/y_ud],
        #     [0, -1/y_ud]
        # ])
        # print([x_rl,y_ud])

        
        F_so_mb, P_so_mb, Po, Cost_so_mb = self.lamda_contractive_LQR_control_model_based(A=self.A, B=self.B, W =self.W, Q=self.Q, R=self.R, M=self.M, lamda=self.lamda)
        # print(F_so_mb)

        # Split the matrix into 2x2 blocks
        P_fourBfour = P_so_mb.copy()
        P11 = P_fourBfour[:2, :2]
        P12 = P_fourBfour[:2, 2:]
        P21 = P_fourBfour[2:, :2]
        P22 = P_fourBfour[2:, 2:]
        P_twoBtwo = P11 - np.dot(np.dot(P12, np.linalg.inv(P22)), P21)
        return P_twoBtwo, F_so_mb, border
       

    def connect(self,n1,n2):
        (x1,y1) = (self.x_in_pixels[n1],self.y_in_pixels[n1])
        (x2,y2) = (self.x_in_pixels[n2],self.y_in_pixels[n2])
        if self.crossObstacle(x1,x2,y1,y2):
            self.remove_node(n2)
        else:
            self.add_edge(n1,n2)
            return True
       

    def step(self, nnear, nrand):

        dmax = self.goal_radius
        
        (xnear, ynear) = (self.x[nnear], self.y[nnear])
        (xrand, yrand) = (self.x[nrand], self.y[nrand])

        self.remove_node(nrand)

        if abs(xrand - self.goal[0]) <= dmax and abs(yrand - self.goal[1]) <= dmax:
            self.add_node(nrand, xrand, yrand)
            self.goalstate = nrand
            self.goalFlag = True
        else:
            self.add_node(nrand, xrand, yrand)

    # def bias(self, ngoal): #TODO develop
    #     n = self.number_of_nodes()
    #     self.add_node(n, ngoal[0], ngoal[1])


    #     nnear = self.nearest(n)
    #     self.step(nnear, n)
    #     self.connect(nnear, n)
    #     return self.x, self.y, self.parent

    def expand(self):

        SUC = 0

        n = self.number_of_nodes()
        x, y = self.sample_envir()

        self.add_node(n, x, y)

        if (self.isFree()):

            P_so_mb, F_so_mb,border_ =self.ellipsoid_gen(x, y)

            if(self.any_within_ellips(P_so_mb,x,y)):
                SUC = 1
                xnearest = self.nearest_within_ellips(P_so_mb,x,y)
                self.Ps.insert(n,P_so_mb)
                self.Ks.insert(n,F_so_mb)
                self.borders.insert(n,border_)
                self.step(xnearest, n)
                self.connect(xnearest, n)

        return self.x, self.y, self.parent,self.Ps, self.borders, SUC

    def expand_from_goal(self):

        SUC = 0

        n = self.number_of_nodes()
        x, y = self.sample_envir_from_goal()

        self.add_node(n, x, y)

        if (self.isFree()):

            P_so_mb, F_so_mb,border_ =self.ellipsoid_gen(x, y)

            if(self.any_within_ellips(P_so_mb,x,y)):
                SUC = 1
                xnearest = self.nearest_within_ellips(P_so_mb,x,y)
                self.Ps.insert(n,P_so_mb)
                self.Ks.insert(n,F_so_mb)
                self.borders.insert(n,border_)
                self.step(xnearest, n)
                self.connect(xnearest, n)

        return self.x, self.y, self.parent,self.Ps, self.borders, SUC

      

    def path_to_goal(self):
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalstate)
            newpos = self.parent[self.goalstate]
            while (newpos != 0):
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
            self.path.reverse()
        return self.goalFlag

    def getPathCoords_in_pixels(self):
        pathCoords = []
        # ellipsoids = []
        for node in self.path:
            x, y = (self.x_in_pixels[node], self.y_in_pixels[node])
            pathCoords.append((x, y))
            # ellipsoids.append(self.Ps[node])
        return pathCoords


    def getPathCoords(self):
        pathCoords = []
        # ellipsoids = []
        for node in self.path:
            x, y = (self.x[node], self.y[node])
            pathCoords.append((x, y))
            # ellipsoids.append(self.Ps[node])
        return pathCoords


    def getEllipsoid_along_path(self):
        Ps_along_path = []
        for node in self.path:
            Ps_along_path.append(self.Ps[node])

        return Ps_along_path

    def getGains_along_path(self):
        Gs_along_path = []
        for node in self.path:
            Gs_along_path.append(self.Ks[node])

        return Gs_along_path



    def cost(self):
        pass

    def compute_gain(self):
        pass
