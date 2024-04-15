from turtle import width
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

from RRT_BASE_EL_spaceCraft import RRTGraph
from RRT_BASE_EL_spaceCraft import RRTMap
import time
from scipy.signal import cont2discrete
from control import dlqr


Real_MapDimension = (50,50)
dimensions = (800,800)

min_x = -Real_MapDimension[0]
max_x = Real_MapDimension[0]
min_y = -Real_MapDimension[1]
max_y = Real_MapDimension[1] 


Maph = dimensions[0]
Mapw = dimensions[1]

start = (-30,30)
goal = (30,-30)
obsdim = 16
obsnum = 1

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
BOUNDARY_COLOR = (150, 190, 30)  



def map_to_screen(x_meters, y_meters):

    screen_width = Mapw
    screen_height = Maph

    # Calculate the scale factors for conversion
    x_scale = screen_width / (max_x - min_x)
    y_scale = screen_height / (max_y - min_y)
    
    x_pixels = (x_meters - min_x) * x_scale
    y_pixels = (-y_meters + max_y) * y_scale

    return int(x_pixels), int(y_pixels)

# Draw axes with numbers
def draw_axes(screen,HEIGHT=Maph,WIDTH=Mapw):
    # Draw x-axis
    pygame.draw.line(screen, BLACK, (0, HEIGHT - 50), (WIDTH, HEIGHT - 50), 2)
    for i in range(-50, 51, 10):
        x = i * (WIDTH / 100) + WIDTH // 2
        pygame.draw.line(screen, BLACK, (x, HEIGHT - 55), (x, HEIGHT - 45), 2)
        font = pygame.font.Font(None, 35)
        text = font.render(str(i) + "", True, BLACK)
        text_rect = text.get_rect(center=(x, HEIGHT - 70))
        screen.blit(text, text_rect)

    # Draw y-axis
    pygame.draw.line(screen, BLACK, (50, 0), (50, HEIGHT), 2)
    for i in range(-50, 51, 10):
        y = HEIGHT // 2 - i * (HEIGHT / 100)
        pygame.draw.line(screen, BLACK, (45, y), (55, y), 2)
        font = pygame.font.Font(None, 35)
        text = font.render(str(i) + "", True, BLACK)
        text_rect = text.get_rect(center=(30, y))
        screen.blit(text, text_rect)

    # Add x and y axis labels at the centers
    font = pygame.font.Font(None, 40)
    x_label = font.render("x (m)", True, BLACK)
    x_label_rect = x_label.get_rect(center=(WIDTH // 2, HEIGHT - 30))
    screen.blit(x_label, x_label_rect)
    
    y_label = font.render("y (m)", True, BLACK)
    y_label_rect = y_label.get_rect(center=(90, HEIGHT // 2))
    screen.blit(y_label, y_label_rect)

def inside_ellipsoidal(P,x_c,y_c,x,y):
    P = np.array(P)
    # Calculate the ellipsoid equation for each point
    z = (x-x_c)**2 * P[0,0] + (y-y_c)**2 * P[1,1] + 2 * (x-x_c) * (y-y_c) * P[0,1]
    if(z>1.0):
        return False
    else:
        return True

def cost(state_,input_,Q,R):

    return np.dot(np.dot(state_.T, Q), state_) + np.dot(np.dot(input_.T, R), input_)


def drawEllip(map,color,x_c,y_c,Pss):

    # print(x_c,y_c,Pss)

    # Define the range for x and y
    x_range = np.linspace(min_x, max_x, Mapw)
    y_range = np.linspace(min_y, max_y, Maph)

    # Create a grid of points
    x, y = np.meshgrid(x_range, y_range)

    P = np.array(Pss)
    # Calculate the ellipsoid equation for each point
    z = (x-x_c)**2 * P[0,0] + (y-y_c)**2 * P[1,1] + 2 * (x-x_c) * (y-y_c) * P[0,1]

    # Draw the contour lines of the ellipsoid
    for level in [1.0]:
        for i in range(Mapw):
            for j in range(Maph):
                if abs(z[j, i] - level) < 0.01:
                    x_pixel, y_pixel = map_to_screen(x[j, i], y[j, i])
                    pygame.draw.circle(map, color, (x_pixel, y_pixel), 1)

def computeUbar(x,y,A,B):
    n, m = B.shape
    c = np.array([[1,0,0,0],[0,1,0,0]])
    M1 = np.hstack([A-np.eye(n),B])
    M2 = np.hstack([c,np.zeros((2,2))])

    M = np.vstack([M1,M2])
    r = np.array([[0],[0],[0],[0],[x],[y]])
    mid = np.linalg.inv(M)@r

    uBar = np.array([mid[-2],mid[-1]])

    return uBar

def main():

    # Define the matrices of the system
    n_ = 1.1e-1

    A = np.array([
    [0, 0, 1, 0],
    [0,0,0,1],
    [3*n_**2,0,0,2*n_],
    [0,0,-2*n_,0]
    ])

    B = np.array([
        [0, 0],
        [0, 0],
        [1,0],
        [0,1]
    ])

    sample_time = 30
    C = np.array([[1,0,0,0],[0,1,0,0]])
    # Assuming you have your continuous-time state-space matrices A, B, and C
    d_system = cont2discrete((A, B,C,0), sample_time, method='zoh')
    A,B,C = d_system[0],d_system[1],d_system[2]
    # print(d_system)
    # exit()
  

    # Noise covariance
    # W = 0.0*np.array([
    #     [0.0100, 0.003],
    #     [0.003, 0.020]
    # ])

    W = 0

    # State and input dimensions
    n, m = B.shape

    # Contractivity factor
    lamda = 0.95

    # LQR parameters
    R = 10e2 *np.eye(2)
    Q = np.array([
        [1e-4, 0,0,0],
        [0, 1e-4,0,0],
        [0, 0,1e2,0],
        [0, 0,0,1e2]
        ])

    # R = 10e1 *np.eye(2)
    # Q = np.array([
    #     [100, 0,0,0],
    #     [0, 10,0,0],
    #     [0, 0,1,0],
    #     [0, 0,0,1]
    #     ])

        
    goal_radius=5
    tol = 0.2
    iteration = 0
    pygame.init()
    map = RRTMap(start,goal, dimensions,Real_MapDimension,obsdim, obsnum,goal_radius=goal_radius)
    graph = RRTGraph(start,goal,dimensions,Real_MapDimension,obsdim, obsnum, A, B, W, Q, R, lamda,goal_radius=goal_radius)

    obstacles = graph.makeObs_fromMeters_at_center()
    map.drawMap(obstacles)
    draw_axes(map.map)
    pygame.draw.rect(map.map, BOUNDARY_COLOR, (0, 0, Maph, Mapw), 8)

    clock = pygame.time.Clock()


    while(not graph.path_to_goal()):
        print(f"Iteration: {iteration}", end='\r', flush=True)
        if iteration%10==0:
            X, Y, Parent, P, borders_, SUC = graph.expand_from_goal()     
        else:
            X, Y, Parent, P, borders_, SUC = graph.expand() 

        if SUC == 1:
            # border_ = borders_[-1]
            x_node, y_node = map_to_screen(X[-1],Y[-1])
            x_parent, y_parent = map_to_screen(X[Parent[-1]],Y[Parent[-1]])
            pygame.draw.circle(map.map,map.Red,(x_node,y_node),map.nodeRad,map.nodeThickness)
            pygame.draw.line(map.map,map.Blue,(x_node,y_node),(x_parent, y_parent),map.edgeThickness)
            # print(P[-1])


            # drawEllip(map.map,map.elColor,X[-1],Y[-1], P[-1])


            # print(border_)
            # print(borders_)

            # x_b, y_b = map_to_screen(border_[1],border_[2])
            # dim_x_b = (border_[0] - border_[1])*Mapw/(2*max_x)
            # dim_y_b = (border_[2] - border_[3])*Maph/(2*max_y)
            # rectang_b=pygame.Rect((x_b, y_b),(dim_x_b ,dim_y_b))
            # pygame.draw.rect(map.map,map.Blue,rectang_b,3)

        if iteration%1 ==0:
            pygame.display.update()
        iteration = iteration + 1

    map.drawPath(graph.getPathCoords_in_pixels(),graph.getEllipsoid_along_path())


    gains = graph.getGains_along_path()
    centers = graph.getPathCoords()
    ellipsoids = graph.getEllipsoid_along_path()
    # print(centers[1])
    # print(gains)

    
    loc_now = [centers[0][0],centers[0][1],0,0]
    loc_now = np.array(loc_now).reshape((n, 1))
    loc_final = [centers[-1][0],centers[-1][1],0,0]
    loc_final = np.array(loc_final).reshape((n, 1))
    final_loc = np.array([[centers[-1][0]],[centers[-1][1]]])

    cost_soda = 0 
    
    h = 1
    # exit()
    
    while (h<len(centers)):

        x_old = loc_now[0][0]
        y_old = loc_now[1][0]

        for e in range(h,len(ellipsoids)):
            if(inside_ellipsoidal(ellipsoids[e],centers[e][0],centers[e][1],x_old,y_old)):
                h = e

        loc_next = [centers[h][0],centers[h][1],0,0]
        loc_next = np.array(loc_next).reshape((n, 1))

        U = gains[h]@(loc_now-loc_next) + computeUbar(loc_next[0],loc_next[1],A,B)
        cost_soda = cost_soda+cost(loc_now-loc_final,U,Q,R)

        loc_now = A@(loc_now) + B@U

        x_new = loc_now[0][0]
        y_new = loc_now[1][0]

        x_new_pixel, y_new_pixel=map_to_screen(x_new,y_new)
        x_old_pixel, y_old_pixel=map_to_screen(x_old,y_old)
        # pygame.draw.circle(map.map,map.colors[h+1],(x_new_pixel,y_new_pixel),map.nodeRad+5)
        now_loc = np.array([x_new,y_new])
        pygame.draw.line(map.map,map.colors[h],(x_old_pixel,y_old_pixel),(x_new_pixel, y_new_pixel),map.edgeThickness+3)
        pygame.display.update()   
        print(np.linalg.norm(now_loc-final_loc))

        if(np.linalg.norm(now_loc-final_loc) <=tol):
            break

    print("\nSODA done and the cost is")
    pygame.image.save(map.map, "SODA-RRT.png")
    print("{:.3e}".format(cost_soda[0][0][0])) 
    input("Press Enter to continue... ")

    pygame.init()
    map_2 = RRTMap(start,goal, dimensions,Real_MapDimension,obsdim, obsnum,goal_radius=goal_radius,MapwindowName="LQR")
    map_2.drawMap(obstacles)
    pygame.draw.rect(map_2.map, BOUNDARY_COLOR, (0, 0, Maph, Mapw), 8)

    map_2.drawPath(graph.getPathCoords_in_pixels(),[])
    draw_axes(map_2.map)


    loc_now = [centers[0][0],centers[0][1],0,0]
    loc_now = np.array(loc_now).reshape((n, 1))
    loc_final = [centers[-1][0],centers[-1][1],0,0]
    loc_final = np.array(loc_final).reshape((n, 1))
    
    final_loc = np.array([[centers[-1][0]],[centers[-1][1]]])
    now_loc = np.array([centers[0][0],centers[0][1]])
    
    h = 1

    K, S, E = dlqr(A, B, Q, R)
    print("\n The optimal Control Gain is:")
    print(K)


    loc_next = [centers[h][0],centers[h][1],0,0]
    loc_next = np.array(loc_next).reshape((n, 1))
    next_loc = np.array([[centers[h][0]],[centers[h][1]]])
    # print(next_loc)
    cost_lqr = 0


    while (h<len(centers)):

        x_old = loc_now[0][0]
        y_old = loc_now[1][0]

        if(np.linalg.norm(now_loc-next_loc) <=tol):
            h = h+1

        loc_next = [centers[h][0],centers[h][1],0,0]
        loc_next = np.array(loc_next).reshape((n, 1))
        next_loc = np.array([[centers[h][0]],[centers[h][1]]])

        # print(now_loc)
        # print(next_loc)

        U = -K@(loc_now-loc_next) + computeUbar(loc_next[0],loc_next[1],A,B)
        cost_lqr = cost_lqr+cost(loc_now-loc_final,U,Q,R)

        loc_now = A@(loc_now) + B@U

        x_new = loc_now[0][0]
        y_new = loc_now[1][0]

        x_new_pixel, y_new_pixel=map_to_screen(x_new,y_new)
        x_old_pixel, y_old_pixel=map_to_screen(x_old,y_old)
        # pygame.draw.circle(map.map,map.colors[h+1],(x_new_pixel,y_new_pixel),map.nodeRad+5)
        now_loc = np.array([x_new,y_new])
        pygame.draw.line(map_2.map,map_2.colors[4],(x_old_pixel,y_old_pixel),(x_new_pixel, y_new_pixel),map_2.edgeThickness+3)
        pygame.display.update()   
        # print(np.linalg.norm(now_loc-final_loc))

        if(np.linalg.norm(now_loc-final_loc) <=tol):
            break

    print("\n SODA-RRT LQR done and the cost is:")
    print("{:.3e}".format(cost_lqr[0][0][0])) 


    loc_now = [centers[0][0],centers[0][1],0,0]
    loc_now = np.array(loc_now).reshape((n, 1))
    loc_final = [centers[-1][0],centers[-1][1],0,0]
    loc_final = np.array(loc_final).reshape((n, 1))



    final_loc = np.array([[centers[-1][0]],[centers[-1][1]]])
    now_loc = np.array([centers[0][0],centers[0][1]])
    
    h = len(centers)-1

    loc_next = [centers[h][0],centers[h][1],0,0]
    loc_next = np.array(loc_next).reshape((n, 1))
    next_loc = np.array([[centers[h][0]],[centers[h][1]]])

    cost_lqr_main = 0


    while (h<len(centers)):

        x_old = loc_now[0][0]
        y_old = loc_now[1][0]

        loc_next = [centers[h][0],centers[h][1],0,0]
        loc_next = np.array(loc_next).reshape((n, 1))
        next_loc = np.array([[centers[h][0]],[centers[h][1]]])

        # print(now_loc)
        # print(next_loc)

        U = -K@(loc_now-loc_next) + computeUbar(loc_next[0],loc_next[1],A,B)
        cost_lqr_main = cost_lqr_main+cost(loc_now-loc_final,U,Q,R)

        loc_now = A@(loc_now) + B@U

        x_new = loc_now[0][0]
        y_new = loc_now[1][0]

        x_new_pixel, y_new_pixel=map_to_screen(x_new,y_new)
        x_old_pixel, y_old_pixel=map_to_screen(x_old,y_old)
        # pygame.draw.circle(map.map,map.colors[h+1],(x_new_pixel,y_new_pixel),map.nodeRad+5)
        now_loc = np.array([x_new,y_new])
        pygame.draw.line(map_2.map,map_2.colors[17],(x_old_pixel,y_old_pixel),(x_new_pixel, y_new_pixel),map_2.edgeThickness+3)
        pygame.display.update()   
        # print(np.linalg.norm(now_loc-final_loc))

        if(np.linalg.norm(now_loc-final_loc) <=tol):
            break



    print("\n LQR done and the cost is:")
    print("{:.3e}".format(cost_lqr_main[0][0][0])) 
    pygame.image.save(map_2.map, "SODA-RRT LQR and LQR.png")

    # update display

    running = True
    while running == True:
        pygame.display.update()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_q:
                running = False
    
    pygame.quit()

if __name__=='__main__':
    main()