from dash import Dash, dcc, html, Input, Output, State, callback_context
from dash.exceptions import PreventUpdate
from tqdm import tnrange
import plotly.express as px
import pandas as pd
import plotly.graph_objects as go
import plotly.offline as iplot
import plotly as py
import matplotlib.pyplot as plt
import numpy as np
from scipy.special import ellipkinc
import scipy.integrate as integrate
import scipy.special as special
from scipy.optimize import minimize
from time import sleep
from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize_scalar

import time as time

# MODEL
def strip_position_z(t, l, k):
    return l/2 * (1 - special.ellipkinc(t, np.power(k, 2), out=None)/special.ellipk(np.power(k, 2), out=None))

def strip_position_y(t, h):
    return h * np.sin(t)

def troposkein_length(k, L):
    #print("k = %.4f" %k)
    A = 2 / (1 - np.power(k, 2))
    E_k = special.ellipe(np.power(k, 2), out=None)
    #print("E(k) = %.4f" %E_k)
    K_k = special.ellipk(np.power(k, 2), out=None)
    #print("K(k) = %.4f" %K_k)
    S = L * (A * (E_k/K_k) - 1)
    return S

def troposkein_height(k, L):
    K_k = special.ellipk(np.power(k, 2), out=None)
    k_ratio = k/((1 - np.power(k, 2)) * K_k)
    #print("K(k) = %.4f" %K_k)
    H = L * k_ratio
    return H

def middle_tension(k, height, rad_s, kg_m):
    #tension = (kg_m * np.power(rad_s, 2) *  np.power(height, 2)) / 4.0
    #tension *= (1 - np.power(k, 2))/np.power(k, 2)
    tension = kg_m * np.power(rad_s, 2) *  np.power(height, 2)
    #print("k:%.4f |  height:%.4fm | speed:%0.4frad_s | mass:%.4fkg_m ->%.4fN" %(k,height, rad_s, kg_m, tension))
    return tension

def edge_tension(k, y, rad_s, kg_m):
    mid_y_index = len(y)/2
    mid_y_index = int(mid_y_index + 1) if mid_y_index % 2 > 0 else int(mid_y_index)
    dys = np.diff(y[0:mid_y_index], 1)
    edge_tension = middle_tension(k, np.max(y), rad_s, kg_m)
    for i in range(len(dys)):
        c_y = y[i]
        d_dy = dys[i]
        added_tension = kg_m * np.power(rad_s, 2) * c_y * d_dy
        edge_tension += added_tension
    return edge_tension


def get_strip(l, k, h):
    t = np.linspace(0, np.pi, num=100)
    z = strip_position_z(t, l, k)
    y = strip_position_y(t, h)
    return (z, y)

def troposkein_line(l, k, h, num=100):
    t = np.linspace(0, np.pi, num=num)
    z = strip_position_z(t, l, k)
    y = strip_position_y(t, h)
    return (z, y)

def find_k(k, length, height):
    height_length_ratio =  height/length
    K_k = special.ellipk(np.power(k, 2), out=None)
    k_ratio = k/((1 - np.power(k, 2)) * K_k)
    distance = abs(height_length_ratio - k_ratio)
    #print("k:%.3f |  height:%.3f | length:%.3f -> height_length_ratio:%.3f | k_ratio:%.3f (%.3f) " %(k, height, length, height_length_ratio, k_ratio, distance))
    return distance

def find_k_bis(k, s, l):
    new_s = troposkein_length(k, l)
    distance = abs(new_s - s)
    #print("k:%.3f |  height:%.3f | length:%.3f -> height_length_ratio:%.3f | k_ratio:%.3f (%.3f) " %(k, height, length, height_length_ratio, k_ratio, distance))
    return distance

def troposkein_k(S, L, verbose=False):
    k = 1.0
    #print("Length of the pole (L) = %.2fm" %L)  if verbose else 0
    #print("Length of the strip (S) = %.2fm" %S)  if verbose else 0
    res = minimize_scalar(find_k_bis, args=(S, L), bounds=(0, 1), method='bounded')
    #print("k = %.2f" %res.x)  if verbose else 0
    return res.x

def troposkein_without_rope_height(W, S, L, H_OFF, D, alpha = 1, verbose=True, fig=None, plot=False):
    # pre-compute parameters
    k = 1.0
    print("Length of the pole (L) = %.2fmm" %L)  if verbose else 0
    print("Length of the rope (S) = %.2fmm" %S)  if verbose else 0
    res = minimize_scalar(find_k_bis, args=(S, L), bounds=(0, 1), method='bounded')
    k =  res.x
    print("k = %.2f" %res.x) if verbose else 0
    H = troposkein_height(k, L)
    print("Height of the troposkein (H) = %.2fmm" %H)  if verbose else 0

    # res = minimize_scalar(find_k, args=(L, H), bounds=(0, 1), method='bounded')
    # k =  res.x
    # print("k = %.2f" %res.x) if verbose else 0

    rad_s = W * 0.1047198
    kg_m = D
    total_h_m = H + H_OFF
    mid_tension = middle_tension(k, total_h_m, rad_s, kg_m)
    print("central_tension = %.4fN (%.4fkg)" %(mid_tension, mid_tension * 0.101972)) if verbose else 0
    z, y = get_strip(L, k, H)
    total_length_rope = troposkein_length(k,  L)
    print("estimated_total_length_rope = %.4fm" %(total_length_rope)) if verbose else 0
    fitted_length = np.sum(np.sqrt(np.diff(z)**2 + np.diff(y)**2))
    print("fitted_total_length_rope = %.4fm" %(fitted_length)) if verbose else 0
    y += H_OFF
    # integral
    mid_y_index = len(y)/2
    mid_y_index = int(mid_y_index + 1) if mid_y_index % 2 > 0 else int(mid_y_index)
    dys = np.diff(y[0:mid_y_index], 1)
    edge_tension = mid_tension
    for i in range(len(dys)):
        c_y = y[i]
        d_dy = dys[i]
        added_tension = kg_m * np.power(rad_s, 2) * c_y * d_dy
        edge_tension += added_tension

    print("extremity_tension = %.4fN (%.4fkg)" %(edge_tension, edge_tension * 0.101972)) if verbose else 0
    if plot:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(np.full(z.shape, 0), y, z)
        #ax.plot(z, y)
        ax.set_aspect('equal')
        plt.show()
    return k, z, y


def troposkein_without_rope_curve(W, L, H, H_OFF, D, alpha = 1, verbose=True, fig=None, plot=False):
    # pre-compute parameters
    k = 1.0
    print("Length of the pole (L) = %.2fmm" %L)  if verbose else 0
    print("Height of the strip (H) = %.2fmm" %H)  if verbose else 0
    res = minimize_scalar(find_k, args=(L, H), bounds=(0, 1), method='bounded')
    k =  res.x
    print("k = %.2f" %res.x) if verbose else 0
    S = troposkein_length(k, L)
    print("Length of the strip (S) = %.2fmm" %S)  if verbose else 0

    # res = minimize_scalar(find_k, args=(L, H), bounds=(0, 1), method='bounded')
    # k =  res.x
    # print("k = %.2f" %res.x) if verbose else 0

    rad_s = W * 0.1047198
    kg_m = D
    total_h_m = H + H_OFF
    mid_tension = middle_tension(k, total_h_m, rad_s, kg_m)
    print("central_tension = %.4fN (%.4fkg)" %(mid_tension, mid_tension * 0.101972)) if verbose else 0
    z, y = get_strip(L, k, H)
    total_length_rope = troposkein_length(k,  L)
    print("estimated_total_length_rope_without_offset = %.4fm" %(total_length_rope)) if verbose else 0
    fitted_length = np.sum(np.sqrt(np.diff(z)**2 + np.diff(y)**2))
    print("fitted_total_length_rope_without_offset = %.4fm" %(fitted_length)) if verbose else 0
    y += H_OFF
    # integral
    mid_y_index = len(y)/2
    mid_y_index = int(mid_y_index + 1) if mid_y_index % 2 > 0 else int(mid_y_index)
    dys = np.diff(y[0:mid_y_index], 1)
    edge_tension = mid_tension
    for i in range(len(dys)):
        c_y = y[i]
        d_dy = dys[i]
        added_tension = kg_m * np.power(rad_s, 2) * c_y * d_dy
        edge_tension += added_tension

    print("extremity_tension = %.4fN (%.4fkg)" %(edge_tension, edge_tension * 0.101972)) if verbose else 0
    if plot:
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(z, y, np.full(z.shape, 0))
        ax.set_aspect('equal')
        plt.show()
    return k, z, y

def troposkein3D(W, S, L, O, D, P, T, verbose=True):
    global max_vertical_resolution, max_horizontal_resolution, fitted_length, fitted_height, max_diameter, max_perimeter, pixel_density, pixel_pitch
    # pre-compute parameters
    # pre-compute parameters
    print("################################")  if verbose else 0
    He_offset = O
    Se_offset = L / 2
    Se_top_offset = T/P
    print("Radius of the hole (O) = %.2f mm" %He_offset)  if verbose else 0
    print("Length of the pole (L) = %.2f mm" %L)  if verbose else 0
    print("Length of the strip (S) = %.2f mm" %S)  if verbose else 0
    k = troposkein_k(S, L, verbose=False)
    print("k = %.2f" %k)  if verbose else 0
    He = troposkein_height(k, L)
    print("Estimated height of the troposkein (He) = %.2f mm" %He)  if verbose else 0
    Se = troposkein_length(k,  L)
    print("Estimated length of the strip (Se) = %.2f mm" %Se)  if verbose else 0
    z, y = troposkein_line(L, k, He, num=1000)
    Sd = np.sum(np.sqrt(np.diff(z)**2 + np.diff(y)**2))
    print("Discretized length of the strip (Sd) = %.2f mm" %Sd)  if verbose else 0
    # tension
    He_total = (He + He_offset) 
    z += Se_offset
    y += He_offset
    rad_s = W * 0.1047198
    kg_m = D
    central_tension = middle_tension(k, He_total/1000, rad_s, kg_m)
    print("central_tension = %.4f N (%.4f kg)" %(central_tension, central_tension * 0.101972)) if verbose else 0
    extremity_tension = edge_tension(k, y/1000, rad_s, kg_m)
    print("extremity_tension = %.4f N (%.4f kg)" %(extremity_tension, extremity_tension * 0.101972)) if verbose else 0

    max_vertical_resolution = np.floor(Sd/P)
    print("max vertical resolution = %.2f pixels" %max_vertical_resolution)  if verbose else 0
    max_diameter = np.max(y) * 2
    print("max diameter = %.2f mm" %max_diameter)   if verbose else 0
    max_perimeter = max_diameter * np.pi
    print("max perimeter = %.2f mm" %max_perimeter)   if verbose else 0
    max_horizontal_resolution = int(np.floor(max_perimeter/P)) 
    print("max horizontal resolution = %.2f pixels" %max_horizontal_resolution)   if verbose else 0
    pixel_density = max_vertical_resolution/Sd # nb leds for length
    print("pixel density = %.2f pixels/mm" %pixel_density)  if verbose else 0
    pixel_pitch = Sd/max_vertical_resolution # nb leds for length
    print("pixel pitch = %.2f mm" %pixel_pitch)  if verbose else 0

    
    final_pixels_x = np.array([])
    final_pixels_y = np.array([])
    final_pixels_z = np.array([])

    pixels_x = np.array([])
    pixels_y = np.array([])
    pixels_z = np.array([])
    leds_count = 0
    t = np.linspace(0, np.pi, num=max_horizontal_resolution)
    for i in range(len(y)):
        current_length =  np.sum(np.sqrt(np.diff(z[:i])**2 + np.diff(y[:i])**2))
        diff_led_position = current_length - leds_count * pixel_pitch
        if diff_led_position > 0 :
                if leds_count > Se_top_offset:
                    pixels_z =  np.insert(pixels_z, len(pixels_z), z[i])
                    pixels_y = np.insert(pixels_y, len(pixels_y), y[i])
                leds_count += 1

    pixels_x = np.full(pixels_y.shape, 0.0)
    pixels_vector = np.array([pixels_x, pixels_y, pixels_z]).T


    for dd in np.arange(0, 360, 360/max_horizontal_resolution):
        r =  R.from_euler('z', dd, degrees=True)
         
        new_pixels_vector =  r.apply(pixels_vector)
        final_pixels_x = np.insert(final_pixels_x, len(final_pixels_x), new_pixels_vector[:, 0])
        final_pixels_y = np.insert(final_pixels_y, len(final_pixels_y), new_pixels_vector[:, 1])
        final_pixels_z = np.insert(final_pixels_z, len(final_pixels_z), new_pixels_vector[:, 2])

    final_pixels_x = final_pixels_x.flatten()
    final_pixels_y = final_pixels_y.flatten()
    final_pixels_z = final_pixels_z.flatten()

    return  final_pixels_x, final_pixels_y, final_pixels_z, max_diameter, max_vertical_resolution, max_horizontal_resolution, central_tension, extremity_tension


def troposkein2D(W, S, L, O, D, P, T, verbose=True):
    global max_vertical_resolution, max_horizontal_resolution, fitted_length, fitted_height, max_diameter, max_perimeter, pixel_density, pixel_pitch
    # pre-compute parameters
    # pre-compute parameters
    print("################################")  if verbose else 0
    He_offset = O
    Se_offset = L / 2
    Se_top_offset = T/P
    print("Radius of the hole (O) = %.2f mm" %He_offset)  if verbose else 0
    print("Length of the pole (L) = %.2f mm" %L)  if verbose else 0
    print("Length of the strip (S) = %.2f mm" %S)  if verbose else 0
    k = troposkein_k(S, L, verbose=False)
    print("k = %.2f" %k)  if verbose else 0
    He = troposkein_height(k, L)
    print("Estimated height of the troposkein (He) = %.2f mm" %He)  if verbose else 0
    Se = troposkein_length(k,  L)
    print("Estimated length of the strip (Se) = %.2f mm" %Se)  if verbose else 0
    z, y = troposkein_line(L, k, He, num=1000)
    Sd = np.sum(np.sqrt(np.diff(z)**2 + np.diff(y)**2))
    print("Discretized length of the strip (Sd) = %.2f mm" %Sd)  if verbose else 0
    # tension
    He_total = (He + He_offset) 
    z += Se_offset
    y += He_offset
    rad_s = W * 0.1047198
    kg_m = D
    central_tension = middle_tension(k, He_total/1000, rad_s, kg_m)
    print("central_tension = %.4f N (%.4f kg)" %(central_tension, central_tension * 0.101972)) if verbose else 0
    extremity_tension = edge_tension(k, y/1000, rad_s, kg_m)
    print("extremity_tension = %.4f N (%.4f kg)" %(extremity_tension, extremity_tension * 0.101972)) if verbose else 0

    max_vertical_resolution = np.floor(Sd/P)
    print("max vertical resolution = %.2f pixels" %max_vertical_resolution)  if verbose else 0
    max_diameter = np.max(y) * 2
    print("max diameter = %.2f mm" %max_diameter)   if verbose else 0
    max_perimeter = max_diameter * np.pi
    print("max perimeter = %.2f mm" %max_perimeter)   if verbose else 0
    max_horizontal_resolution = int(np.floor(max_perimeter/P)) 
    print("max horizontal resolution = %.2f pixels" %max_horizontal_resolution)   if verbose else 0
    pixel_density = max_vertical_resolution/Sd # nb leds for length
    print("pixel density = %.2f pixels/mm" %pixel_density)  if verbose else 0
    pixel_pitch = Sd/max_vertical_resolution # nb leds for length
    print("pixel pitch = %.2f mm" %pixel_pitch)  if verbose else 0

    
    final_pixels_y = np.array([])
    final_pixels_z = np.array([])

    pixels_y = np.array([])
    pixels_z = np.array([])
    leds_count = 0
    t = np.linspace(0, np.pi, num=max_horizontal_resolution)
    for i in range(len(y)):
        current_length =  np.sum(np.sqrt(np.diff(z[:i])**2 + np.diff(y[:i])**2))
        diff_led_position = current_length - leds_count * pixel_pitch
        if diff_led_position > 0:
                if leds_count > Se_top_offset:
                    pixels_z =  np.insert(pixels_z, len(pixels_z), z[i])
                    pixels_y = np.insert(pixels_y, len(pixels_y), y[i])
                leds_count += 1


    final_pixels_y = pixels_y.flatten()
    final_pixels_z = pixels_z.flatten()

    return  final_pixels_y, final_pixels_z, max_diameter, max_vertical_resolution, max_horizontal_resolution, central_tension, extremity_tension


if __name__ == '__main__':
    # L = length of the pole
    # H = maximal height of the rope
    # troposkein_without_rope_curve(W=37.5287355810689, L=50, H=17, H_OFF=0, D=25, alpha = 1, verbose=True, fig=None, plot=True) # set k to 0.45 to reproduce result
    # troposkein_without_rope_curve(W=380, H=0.20, L=0.25, H_OFF=0.10, D=0.022, alpha = 1, verbose=True, fig=None, plot=True)
    troposkein_without_rope_height(W=350, S=0.215, L=0.195, H_OFF=0.1, D=0.022, alpha = 1, verbose=True, fig=None, plot=True)
    troposkein_without_rope_height(W=350, S=0.415, L=0.195, H_OFF=0.1, D=0.022, alpha = 1, verbose=True, fig=None, plot=True)
    
