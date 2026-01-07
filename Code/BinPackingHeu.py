#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Aug  3 18:05:39 2023

@author: chunlongyu
"""
import os
import sys
import ctypes
import platform
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import random

# --- 新增这个函数：用于获取资源的绝对路径 ---
def get_resource_path(relative_path):
    """
    获取资源的绝对路径。
    PyInstaller 打包后，文件会被解压到 sys._MEIPASS 临时目录，
    或者在开发环境中，它就是当前脚本的目录。
    """
    if hasattr(sys, '_MEIPASS'):
        return os.path.join(sys._MEIPASS, relative_path)
    return os.path.join(os.path.abspath("."), relative_path)

# Load the shared library
# my_cpp_lib = ctypes.CDLL(r'D:\毕业设计\For HaoWu\代码和数据\Lite39\Lite39\Example2\x64\Release\Call_Cpp_Python_Exp.dll')  # Modify for Linux if needed
my_cpp_lib = ctypes.CDLL(r'D:\毕业设计\For HaoWu\代码和数据\Lite39\Lite39\Bestfit.dll')  # Modify as your path


# Define the function signature for the Print function
my_cpp_lib.BestFitRotate.argtypes = (ctypes.POINTER(ctypes.c_float), ctypes.c_int, ctypes.c_float)
my_cpp_lib.BestFitRotate.restype = ctypes.POINTER(ctypes.c_float)
my_cpp_lib.FirstFitRotate.argtypes = (ctypes.POINTER(ctypes.c_float), ctypes.c_int, ctypes.c_float)
my_cpp_lib.FirstFitRotate.restype = ctypes.POINTER(ctypes.c_float)


def BestFitRotate(bin_sizes, mac_width):
    num_bins = len(bin_sizes) // 2

    # Convert Python list to C array
    c_bin_sizes = (ctypes.c_float * len(bin_sizes))(*bin_sizes)

    # Call the C++ function
    item_coor_ptr = my_cpp_lib.BestFitRotate(c_bin_sizes, num_bins, mac_width)

    # Convert the returned pointer to a Python list
    item_coor_list = [item_coor_ptr[i] for i in range(num_bins*4)]

    # # Free the memory allocated in C++
    # my_cpp_lib.free(item_coor_ptr)

    return item_coor_list

def FirstFitRotate(bin_sizes, mac_width):
    num_bins = len(bin_sizes) // 2

    # Convert Python list to C array
    c_bin_sizes = (ctypes.c_float * len(bin_sizes))(*bin_sizes)

    # Call the C++ function
    item_coor_ptr = my_cpp_lib.FirstFitRotate(c_bin_sizes, num_bins, mac_width)

    # Convert the returned pointer to a Python list
    item_coor_list = [item_coor_ptr[i] for i in range(num_bins*4)]

    # # Free the memory allocated in C++
    # my_cpp_lib.free(item_coor_ptr)

    return item_coor_list

def plot_rectangles(Rectangles,W,H):
    # The bin size
    #W: length of the machine platform
    #H: width of machine platform
    tole = 0.1
    Scale = 1.0
    # import the file
    plt.rcParams['figure.dpi'] = 50 
    fig, ax = plt.subplots(figsize = (W,H),sharex=True, sharey=True,
                            tight_layout=True)
    #assert(not checkOverlap(Rectangles, Solution))
    plt.xlim(0,W)
    plt.ylim(0,H)
    for i in range(0,len(Rectangles)):
        item_idx = i
        item_width = (Rectangles[i][2] - Rectangles[i][0])/Scale
        item_height = (Rectangles[i][3] - Rectangles[i][1])/Scale
        item_x = Rectangles[i][0]/Scale 
        item_y = Rectangles[i][1]/Scale
        p = patches.Rectangle(
        (item_x, item_y), item_width - tole, item_height - tole,
        fill=True, clip_on=False, label = "Item"+str(item_idx),color= np.random.rand(1,3).flatten(),
        )
        ax.add_patch(p)
    plt.legend()
    plt.show()
    
def obj_fun(x, bin_sizes,mac_width):
    # Replace this with your actual objective function
    # It should return a numerical value that you want to minimize.
    # Example: return sum([abs(x[i] - x[i+1]) for i in range(len(x)-1)])
    num_bins = len(x)
    seq_bin_sizes = []
    for i in x:
        seq_bin_sizes.append(bin_sizes[2*i])
        seq_bin_sizes.append(bin_sizes[2*i+1])
    
    coordinates = FirstFitRotate(seq_bin_sizes, mac_width)
    max_height = max(coordinates)
    return max_height
    

def swap_positions(lst, pos1, pos2):
    # Helper function to swap two elements in a list
    lst[pos1], lst[pos2] = lst[pos2], lst[pos1]

def hill_climbing(initial_solution, max_iterations, bin_sizes, mac_width):
    current_solution = initial_solution
    current_cost = obj_fun(current_solution, bin_sizes, mac_width)
    hist_cost =[]

    for i in range(max_iterations):
        # Generate a neighboring solution by randomly swapping two elements in the permutation
        neighbor_solution = current_solution.copy()
        pos1, pos2 = random.sample(range(len(neighbor_solution)), 2)
        swap_positions(neighbor_solution, pos1, pos2)

        # Calculate the cost of the neighboring solution
        neighbor_cost = obj_fun(neighbor_solution, bin_sizes, mac_width)
        
        # If the neighboring solution is better, accept it as the current solution
        if neighbor_cost < current_cost:
            current_solution = neighbor_solution
            current_cost = neighbor_cost
            
        hist_cost.append(current_cost)

    return current_solution, current_cost,hist_cost



# check the feasibility of a given sol
def Sol_feas_check(sol, Ins):
    #print("hello")
    Batches_coor = {}
    P_coor_dict = {}           # Part coordinate dictionary
    P_o_dict = {}              # Part rotation binary dictionary
    UnPackParts = []           # List of unpacked parts
    Batches = sol.Batches
    
    keys = [key for key in Batches]
    keys.sort()
    
    for batch in keys:
        mac_id = batch[0]
        b = batch[1]
        parts = Batches[batch][:]
        
        # remove parts that cannot fit into the platform, i.e., 
        # not( ( part_length <= mac_length & part_width <= mac_width)
        #  || ( part_width <= mac_length & part_length <= mac_width) ) 
        
        mac_length = Ins.L[mac_id]
        mac_width = Ins.W[mac_id] 
        for p in parts:
            part_length = sol.Ls[p]
            part_width = sol.Ws[p]
            if not( ( part_length <= mac_length and part_width <= mac_width) or ( part_width <= mac_length and part_length <= mac_width) ):
                UnPackParts.append(p)
                parts.remove(p)
        
        bin_sizes = []
        for p in parts:
            bin_sizes.append( sol.Ls[p] )
            bin_sizes.append( sol.Ws[p] )
        

        Batches_coor[batch] = BestFitRotate(bin_sizes, mac_length)
        
        
        # Plot the bin packing results
        bins_coor = []
        print("Coordinates of the parts:")
        for i in range(len( Batches_coor[batch])//4):
            x1, y1, x2, y2 =  Batches_coor[batch][i*4 : (i+1)*4]
            bins_coor.append([x1,y1,x2,y2])
            p = parts[i]
            P_coor_dict[p] = [x1,y1]
            
            tol = 0.05
            if abs(x2 - x1 - sol.Ls[p])<= tol  and abs(y2-y1 - sol.Ws[p])<=tol:
                P_o_dict[p] = 1
            else:
                P_o_dict[p] = 0
             
            if y2 >  mac_width:
                UnPackParts.append(p)
            
            print(f"Part {i}: x1={x1}, y1={y1}, x2={x2}, y2={y2}")
        max_height = max( Batches_coor[batch] )
        print("max beight: {}".format(max_height))
        plot_rectangles(bins_coor, mac_length, mac_width)
        
        
    # Write bin packing results to sol
    sol.Xs = []
    sol.Ys = []
    sol.Os = []
    for p in range(Ins.num_parts):
        if not( p in UnPackParts):
            sol.Xs.append(  P_coor_dict[p][0]  )
            sol.Ys.append(  P_coor_dict[p][1]  )
            sol.Os.append( P_o_dict[p])
        else:
            sol.Xs.append( -1  )
            sol.Ys.append( -1  )
            sol.Os.append( -1  )
    
    # Modify sol for unpack parts
    sol.UnPackParts = UnPackParts
    # Modify Is, Bs, Ks (part-to-batch allocation, orientation selection)
    for p in UnPackParts:
        sol.Is[p] = -1
        sol.Bs[p] = -1
        sol.Ks[p] = -1
    
    # Modify Batches
    for batch in sol.Batches:
        for p in UnPackParts:
            if p in sol.Batches[batch]:
                sol.Batches[batch].remove(p)
    
    # Modify performance
    if len(UnPackParts)>0:
        sol.P_Batches={} # dictionary of batch processing times, [i,b] is the key
        sol.C_Batches={} # dictionary of the batch completion times, [i,b] is the key
        sol.Tard=[]      # list of tardiness of each part
        sol.C_max =0     # Maximum completion time
        sol.TotTard= 0   # Total tardiness

    
    
    return sol, P_coor_dict, P_o_dict, UnPackParts

# Evaluate the TotTard and C_max of a given sol
# def eval_sol(sol, Ins):
    
