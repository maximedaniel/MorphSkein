import os
from matplotlib import pyplot as plt
import numpy as np
from troposkein import *
from matplotlib.font_manager import FontProperties
from matplotlib.widgets import Slider, Button, CheckButtons
from matplotlib import cm
from matplotlib.colors import rgb2hex
from tqdm import tqdm
import constants
import sys
from scipy.interpolate import  make_interp_spline, BSpline, interp1d
import scipy as sp
import cv2
##plt.switch_backend('TkAgg')
class MarkerUpdater:
    def __init__(self):
        ##for storing information about Figures and Axes
        self.figs = {}

        ##for storing timers
        self.timer_dict = {}

    def add_ax(self, ax, features=[]):
        ax_dict = self.figs.setdefault(ax.figure,dict())
        ax_dict[ax] = {
            'xlim' : ax.get_xlim(),
            'ylim' : ax.get_ylim(),
            'figw' : ax.figure.get_figwidth(),
            'figh' : ax.figure.get_figheight(),
            'scale_s' : 1.0,
            'scale_a' : 1.0,
            'features' : [features] if isinstance(features,str) else features,
        }
        ax.figure.canvas.mpl_connect('draw_event', self.update_axes)

    def update_axes(self, event):

        for fig,axes in self.figs.items():
            if fig is event.canvas.figure:

                for ax, args in axes.items():
                    ##make sure the figure is re-drawn
                    update = True

                    fw = fig.get_figwidth()
                    fh = fig.get_figheight()
                    fac1 = min(fw/args['figw'], fh/args['figh'])


                    xl = ax.get_xlim()
                    yl = ax.get_ylim()
                    fac2 = min(
                        abs(args['xlim'][1]-args['xlim'][0])/abs(xl[1]-xl[0]),
                        abs(args['ylim'][1]-args['ylim'][0])/abs(yl[1]-yl[0])
                    )

                    ##factor for marker size
                    facS = (fac1*fac2)/args['scale_s']

                    ##factor for alpha -- limited to values smaller 1.0
                    facA = min(1.0,fac1*fac2)/args['scale_a']

                    ##updating the artists
                    if facS != 1.0:
                        for line in ax.lines:
                            if 'size' in args['features']:
                                line.set_markersize(line.get_markersize()*facS)

                            if 'alpha' in args['features']:
                                alpha = line.get_alpha()
                                if alpha is not None:
                                    line.set_alpha(alpha*facA)


                        for path in ax.collections:
                            if 'size' in args['features']:
                                path.set_sizes([s*facS**2 for s in path.get_sizes()])

                            if 'alpha' in args['features']:
                                alpha = path.get_alpha()
                                if alpha is not None:
                                    path.set_alpha(alpha*facA)

                        args['scale_s'] *= facS
                        args['scale_a'] *= facA

                self._redraw_later(fig)



    def _redraw_later(self, fig):
        timer = fig.canvas.new_timer(interval=10)
        timer.single_shot = True
        timer.add_callback(lambda : fig.canvas.draw_idle())
        timer.start()

        ##stopping previous timer
        if fig in self.timer_dict:
            self.timer_dict[fig].stop()

        ##storing a reference to prevent garbage collection
        self.timer_dict[fig] = timer




diff_tol = 1e-8
stop_tol = 1e-8
eps_tol = 1e-8

LED_BITS = 24
FPS = 24
max_vertical_resolution = 0
max_horizontal_resolution = 0
max_diameter = 0
tropos_height = 0
max_perimeter = 0
pixel_density = 0
pixel_pitch = 0

all_solve_done = False

def triangular_index (h_value, l_value):
    h_size =  (constants.MAX_POLE_HEIGHT - constants.MIN_POLE_HEIGHT)/constants.STEP_POLE_HEIGHT + 1 # 3
    l_size =  (constants.MAX_ROPE_LENGTH - constants.MIN_ROPE_LENGTH)/constants.STEP_ROPE_LENGTH + 1 # 61
    h_index = (h_value - constants.MIN_POLE_HEIGHT)/constants.STEP_POLE_HEIGHT #0 - 2
    l_index = (l_value - constants.MIN_ROPE_LENGTH)/constants.STEP_ROPE_LENGTH #0 - 60
    h_size = round(h_size)
    l_size = round(l_size)
    h_index = round(h_index)
    l_index = round(l_index)

    sum_h_index = 0
    for i in range(h_index):
         sum_h_index += l_size - i
    final_index = sum_h_index + l_index + h_index
    # print("h_size:%d" %(h_size))
    # print("l_size:%d" %(l_size))
    # print("h_index:%d" %(h_index))
    # print("l_index:%d" %(l_index))
    # print("sum_h_index:%d" %(sum_h_index))
    # print("final_index:%d" %(final_index))
    # print("%d %d %d -> %d" %(h_size, h_index, l_index, final_index))
    # print("%d" %(l_index - h_index ))
    # print("%d" %(sum_h_index + l_index ))
    return final_index

    
def solve_all(event):
     W =  slider_angular_speed.val
     S =  slider_rope_length.val
     L =  slider_pole_height.val
     O =  slider_hole_radius.val
     Pv = slider_pixel_pitch_vertical.val
     Ph = slider_pixel_pitch_horizontal.val
     D = slider_pixel_mass.val/1000.0
     T= slider_top_rope_length_offset.val
     res = ""
     h_params = []
     l_params = []
     vertical_resolution_results = []
     horizontal_resolution_results = []
     h_values = np.arange(constants.MIN_POLE_HEIGHT, constants.MAX_POLE_HEIGHT + 1, constants.STEP_POLE_HEIGHT).astype(int)
     print("nb H values:", len(h_values))
     l_values = np.arange(constants.MIN_ROPE_LENGTH, constants.MAX_ROPE_LENGTH + 1, constants.STEP_ROPE_LENGTH).astype(int)
     print("nb L values:",  len(l_values))
     nb_steps = 0
     prev_triangle_index = -1
     for i in range(len(h_values)):
        h = float(h_values[i])
        next_l_values = l_values[l_values > h]
        nb_steps += len(next_l_values)
        for j in range(len(next_l_values)):
            l = float(next_l_values[j])
            h_params.append(h)
            l_params.append(l)
            S = l
            L = h
            pixels_x, pixels_y, pixels_z,max_diameter, max_vertical_resolution, max_horizontal_resolution, central_tension, extremity_tension = troposkein3D(W, S, L, O, D, Pv, Ph, T, verbose=False)
            vertical_resolution_results.append(int(max_vertical_resolution))
            horizontal_resolution_results.append(int(max_horizontal_resolution))
            triangle_index = triangular_index(h, l)
            #print("%d" %(triangle_index))
            if triangle_index <= prev_triangle_index:
                print("bad index: %d" %(triangle_index))
                exit()
            # else:
            #     print("horizontal_resolution_result: %d" %(horizontal_resolution_results[triangle_index]))
            print("[%d] (W@%dmm, H@%dmm, L@%dmm) -> %dpx" %(triangle_index, O, h, l, max_horizontal_resolution))
            prev_triangle_index = triangle_index
        print("%d  -> %d" %(i, len(next_l_values)))
     print("total triangular indexes:", len(next_l_values))
     print("Done.")
        
     max_vertical_resolution_results = max(vertical_resolution_results)
     max_horizontal_resolution_results = max(horizontal_resolution_results)
     max_total_resolution_results = max_vertical_resolution_results * max_horizontal_resolution_results

     horizontal_resolution_results_str = str(horizontal_resolution_results)
     #print(l_params)
     res += '#define DISPLAY_MAX_HORIZONTAL_RESOLUTION %d\n' %max_horizontal_resolution_results
     res += '#define DISPLAY_MAX_VERTICAL_RESOLUTION %d\n' %max_vertical_resolution_results
     res += '#define DISPLAY_MAX_NB_PIXELS %d\n' %max_total_resolution_results
     res += '#define MIN_H %d\n' %constants.MIN_POLE_HEIGHT
     res += '#define MAX_H %d\n' %constants.MAX_POLE_HEIGHT  
     res += '#define STEP_H %d\n' %constants.STEP_POLE_HEIGHT
     res += '#define NB_H %d\n' %len(h_values)
     res += '#define MIN_L %d\n' %constants.MIN_ROPE_LENGTH  
     res += '#define MAX_L %d\n' %constants.MAX_ROPE_LENGTH  
     res += '#define STEP_L %d\n' %constants.STEP_ROPE_LENGTH
     res += '#define NB_L %d\n' %len(l_values)
     res += '#define NB_MAX_HORIZONTAL_RESOLUTIONS %d\n' %len(horizontal_resolution_results)
     res += 'uint16_t max_horizontal_resolutions [NB_MAX_HORIZONTAL_RESOLUTIONS] = {%s};\n' %(horizontal_resolution_results_str[1:len(horizontal_resolution_results_str)-1])
     f = open("all_numeric_solutions.cpp", "w")
     f.write(res)
     f.close()
     return res
    #  print('uint16_t[] h_params = {%s};' %h_params_str[1:len(h_params_str)-1])   
    #  print('uint16_t[] l_params = {%s};' %l_params_str[1:len(l_params_str)-1])   
    #  print('uint16_t[] vertical_resolutions = {%s};' %vertical_resolution_results_str[1:len(vertical_resolution_results_str)-1])   



# Shoelace formula
def area(p):
    return np.abs(np.cross(p, np.roll(p, 1, axis=0)).sum()) / 2
     
if __name__ == '__main__':
    args = sys.argv[1:]
    enable_3D = True
    hide_axis = False
    image_path = None
    df_observation = None
    poly = False

    if '--image' in args:
        arg_image_index = args.index('--image')
        image_path = args[arg_image_index + 1]
        print('image_path = %s' %image_path)
    
    if '--poly' in args:
        poly = True

    if '--mode' in args:
        arg_mode_index = args.index('--mode')
        if args[arg_mode_index + 1] == '2D':
            enable_3D = False

    print('enable_3D = %s' %enable_3D)
    if '--hide' in args:
        hide_axis = True
    print('hide_axis = %s' %hide_axis)
    if '--observation' in args:
        arg_observation_index = args.index('--observation')
        arg_observation_path = args[arg_observation_index + 1]
        with open(arg_observation_path) as datafile:
            df_observation = pd.read_json(datafile)
    print('df_observation = ', df_observation.shape if df_observation is not None else None)
    fps_array = np.arange(10.0, 25.0, 2.0)
    print('fps_array: %s' %fps_array)
    angular_speed_array = fps_array * 60 / 4
    print('angular_speed_array: %s' %angular_speed_array)
    height_factors = np.arange(1.0, 2.51, 0.5)
    print('height_factors: %s' %height_factors)
    height_values = height_factors * constants.MIN_POLE_HEIGHT
    print('height_values: %s' %height_values)
    length_factors = np.arange(1.0, 4.01, 0.5)
    print('length_factors: %s' %length_factors)
    length_values = length_factors * constants.MIN_ROPE_LENGTH
    print('length_values: %s' %length_values)
    length_values = length_values[ length_values <= constants.MAX_ROPE_LENGTH]
    print('length_values (clamped): %s' %length_values)
    length_values =  length_values - length_values % 7
    print('length_values (to pixel density): %s' %length_values)
    length_factors_corrected =  length_values/ constants.MIN_ROPE_LENGTH
    print('length_factors_corrected (to ratio back): %s' %length_factors_corrected)

    # H_L_init = MIN_ROPE_LENGTH/MIN_POLE_HEIGHT
    # H_L_ratios = np.arange(H_L_init, H_L_init * 4.51, 0.5)
    # print('H_L_ratios: %s' %H_L_ratios)

    # # print('length_factors: %s' %length_factors)
    # # length_values = length_factors * MIN_ROPE_LENGTH
    # # print('length_values: %s' %length_values)
    for height_value in height_values:
            filtered_length_values = length_values[ length_values > height_value]
            print('%f -> %s' %(height_value, filtered_length_values))
            vertical_resolution_delta = (filtered_length_values - constants.MIN_ROPE_LENGTH)/constants.DEFAULT_PIXEL_PITCH
            print('%f -> %s' %(height_value,vertical_resolution_delta))
            rounded_vertical_resolution_delta = np.round(vertical_resolution_delta)
            print('%.2f -> %s' %(height_value,rounded_vertical_resolution_delta))
    #     print('%.2f -> %s' %(height_value,rounded_vertical_resolution_delta))

    my_updater = MarkerUpdater()
    W=constants.DEFAULT_ANGULAR_SPEED
    S=constants.DEFAULT_ROPE_LENGTH
    L=constants.DEFAULT_POLE_HEIGHT
    O=constants.DEFAULT_HOLE_RADIUS
    D=constants.DEFAULT_STRIP_MASS/1000.0
    Pv=constants.DEFAULT_PIXEL_PITCH
    Ph=constants.DEFAULT_PIXEL_PITCH
    T=constants.DEFAULT_TOP_ROPE_LENGTH_OFFSET
    ObsLeftOffset = 0

    if enable_3D:
        pixels_x, pixels_y, pixels_z, max_diameter, max_vertical_resolution, max_horizontal_resolution, central_tension, extremity_tension = troposkein3D(W, S, L, O, D, Pv, Ph, T,  verbose=True)
    else:
        pixels_y, pixels_z, max_diameter, max_vertical_resolution, max_horizontal_resolution, central_tension, extremity_tension = troposkein2D(W, S, L, O, D, Pv, T, verbose=True)
    
    col_labels = ['']
    col_widths = [0.25]
    tropos_height = (max_diameter - 2.0 * O)/2.0
    row_labels = ['Troposkein height', 'Max diameter','Vertical Resolution','Horizontal Resolution', 'Central Tension', 'Extremity Tension']
    table_vals = [
        ['%0.1f mm'%tropos_height],
        ['%0.1f mm'%max_diameter],
        ['%0.1f px'%max_vertical_resolution],
        ['%0.1f px'%max_horizontal_resolution],
        ['%0.3f N (%.4f kg)' %(central_tension, central_tension * 0.101972)],
        ['%0.3f N (%.4f kg)' %(extremity_tension, extremity_tension * 0.101972)]
        ]
    #row_colors = ['red', 'blue', 'yellow']

    ##setting up the figure
    #plt.rcParams["figure.figsize"] = [7.00, 3.50]
    plt.rcParams["figure.autolayout"] = True
    fig = plt.figure()
    ax = None
    scatter_observed = None
    scatter_observed_raw = None
    #scatter_surface_large_observed = None
    fill_polygon_observed = None
    if enable_3D:
        # Use cropped_image for further processing
        ax = fig.add_subplot(111, projection='3d', proj_type = 'ortho')
        ax.set_box_aspect((np.ptp(pixels_x), np.ptp(pixels_y), np.ptp(pixels_z)))

        if image_path is not None:
            if poly:
                image_bgr = cv2.imread(image_path)
                image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
                image_pixels = []
                image_start_v = image_rgb.shape[0]/2.0 - max_vertical_resolution/2.0
                image_start_h = image_rgb.shape[1]/2.0 - max_horizontal_resolution/2.0
                unique_points = np.unique(np.column_stack((pixels_x, pixels_y, pixels_z)), axis=1)
                pixels_x = unique_points[:,0]
                pixels_y = unique_points[:,1]
                pixels_z = unique_points[:,2]
                nb_rows = max_vertical_resolution
                nb_cols = int(len(pixels_x)//max_vertical_resolution)
                triangles = []
                for i in tqdm(range(len(pixels_x))):
                    col = i // max_vertical_resolution
                    row = i % max_vertical_resolution
                    if row < nb_rows - 1 and col < nb_cols :
                        top_left = i % len(pixels_x)
                        top_right = (i + nb_rows) % len(pixels_x)
                        bottom_left = (i + 1) % len(pixels_x)
                        bottom_right = (i + nb_rows + 1) % len(pixels_x)
                        triangles.append([top_left, bottom_left, top_right])
                        triangles.append([bottom_left, bottom_right, top_right])
                        if row >= 0 and row < image_rgb.shape[0] and col >= 0 and col < image_rgb.shape[1]:
                            r, g, b = image_rgb[int(image_start_v + row), int(image_start_h + col)]
                            image_pixels.append(rgb2hex([r/255.0, g/255.0, b/255.0, 1.0], keep_alpha=True))
                            image_pixels.append(rgb2hex([r/255.0, g/255.0, b/255.0, 1.0], keep_alpha=True))
                            # image_pixels = np.append(image_pixels, [r/255.0, g/255.0, b/255.0, 1.0])
                            # image_pixels = np.append(image_pixels, [r/255.0, g/255.0, b/255.0, 1.0])
                        else:
                            image_pixels.append(rgb2hex([0.0, 0.0, 0.0, 0.0], keep_alpha=True))
                            image_pixels.append(rgb2hex([0.0, 0.0, 0.0, 0.0], keep_alpha=True))
                            # image_pixels = np.append(image_pixels, [0.0, 0.0, 0.0, 0.0])
                            # image_pixels = np.append(image_pixels, [0.0, 0.0, 0.0, 0.0])
                scatter_surface = ax.plot_trisurf(pixels_x, pixels_y, pixels_z, triangles=triangles, cmap=None, linewidth=0.2)
                scatter_surface.set_color(image_pixels)
            else:
                image_bgr = cv2.imread(image_path)
                image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
                image_pixels = []
                image_start_v = image_rgb.shape[0]/2.0 - max_vertical_resolution/2.0
                image_start_h = image_rgb.shape[1]/2.0 - max_horizontal_resolution/2.0
                for h in range(max_horizontal_resolution):
                    for v in range(max_vertical_resolution):
                        if v >= 0 and v < image_rgb.shape[0] and h >= 0 and h < image_rgb.shape[1]:
                            r, g, b = image_rgb[int(image_start_v + v), int(image_start_h + h)]
                            image_pixels.append(rgb2hex([r/255.0, g/255.0, b/255.0, 1.0], keep_alpha=True))
                        else:
                            image_pixels.append(rgb2hex([0.0, 0.0, 0.0, 0.0], keep_alpha=True))
                scatter_surface = ax.scatter(pixels_x, pixels_y, pixels_z, s=1) #, c=image_pixels
                scatter_surface.set_color(image_pixels)
        else:
            scatter_surface = ax.scatter(pixels_x, pixels_y, pixels_z, s=1)
    


    else:
        ax = fig.add_subplot(111)
        ax.set_xlim([0, O + constants.MAX_ROPE_LENGTH/2.0 ])
        ax.set_ylim([0, constants.MAX_POLE_HEIGHT])
        ax.set_aspect('equal')
        #ax.set_box_aspect((np.ptp(pixels_y), np.ptp(pixels_z)))
        scatter_surface,  = ax.plot(pixels_y, pixels_z, linewidth=1, markersize=1, marker='o', color='black',)

        # 300 represents number of points to make between T.min and T.ma

        # LengthMillisSmall = np.sum(np.sqrt(np.diff(xMillisSmall)**2 + np.diff(yMillisSmall)**2))
        # print('LengthMillisSmall: %f' %LengthMillisSmall)
        # LengthMillisLargeCalibrated = np.sum(np.sqrt(np.diff(xMillisLargeCalibrated)**2 + np.diff(yMillisLargeCalibrated)**2))
        # print('LengthMillisLargeCalibrated: %f' %LengthMillisLargeCalibrated)
        
        # interp_large_calibrated_func = interp1d(yMillisLargeCalibrated, xMillisLargeCalibrated,  fill_value="extrapolate")
        # yNewMillisLargeCalibrated = pixels_z
        # xNewMillisLargeCalibrated = interp_large_calibrated_func(yNewMillisLargeCalibrated)
        if df_observation is not None:
            xObserved = df_observation.x.to_numpy()  + constants.DEFAULT_HOLE_RADIUS + ObsLeftOffset
            yObserved = df_observation.y.to_numpy()
            interp_observed_func = interp1d(yObserved, xObserved,  fill_value="extrapolate")
            yNewObserved = pixels_z
            xNewObserved = interp_observed_func(yNewObserved)
            stackPixels = np.column_stack((pixels_y, pixels_z))
            stackObserved = np.column_stack((xNewObserved, yNewObserved))
            polygonObserved = np.r_[stackPixels, stackObserved[::-1]]
            fillPolygonObservedArea = area(polygonObserved)
            row_labels.append('Area Error')
            table_vals.append(['%0.3f mm2' %(np.sqrt(fillPolygonObservedArea))])
            fill_polygon_observed, = ax.fill(*polygonObserved.T, alpha=.2)
            scatter_observed,  = ax.plot(xNewObserved, yNewObserved, linewidth=1, markersize=0, marker='o', color='blue',)
            #scatter_observed_raw,  = ax.plot(xObserved, yObserved, linewidth=1, markersize=0, marker='o', color='red',)
    tab = ax.table(cellText=table_vals,
                    colWidths=col_widths,
                    rowLabels=row_labels,
                    colLabels=col_labels,
                    #rowColours=row_colors,
                    loc='upper right',
                    edges='open')
    
    tab.auto_set_font_size(False)
    tab.set_fontsize(8)
    tab.scale(1, 0.75)
    for (row, col), cell in tab.get_celld().items():
        # cell.set_linewidth(0)
        cell.set_text_props(ha="right")
        if col < 0:
            cell.set_text_props(fontproperties=FontProperties(weight='bold'))

    # hide axis if required
    ax.set_axis_off() if hide_axis else ax.set_axis_on()
    tab.set_visible(not hide_axis)


    ##setting up the updater
    my_updater.add_ax(ax, ['size'])  ##line plot, only marker size
    
    # Make checkbuttons to control axis visiblity
    ax_hide_axis = fig.add_axes([0.05, 0.04, 0.20, 0.03])
    check_hide_axis = CheckButtons(ax_hide_axis, ("hide axis",), (hide_axis,))

    def update_hide_axis(check_hide_axis_label):
        hide_axis_values = check_hide_axis.get_status()
        hide_axis = hide_axis_values[0]
        ax.set_axis_off() if hide_axis else ax.set_axis_on()
        tab.set_visible(not hide_axis)
        ax_left_observed_rope_offset.set_visible(not hide_axis)
        ax_top_rope_length_offset.set_visible(not hide_axis)
        ax_angular_speed.set_visible(not hide_axis)
        ax_hole_radius.set_visible(not hide_axis)
        ax_pole_height.set_visible(not hide_axis)
        ax_rope_length.set_visible(not hide_axis)
        ax_pixel_pitch_vertical.set_visible(not hide_axis)
        ax_pixel_pitch_horizontal.set_visible(not hide_axis)
        ax_pixel_mass.set_visible(not hide_axis)

    check_hide_axis.on_clicked(update_hide_axis)


    # Make a horizontal slider to control the frequency.
    ax_left_observed_rope_offset = fig.add_axes([0.75, 0.28, 0.20, 0.03])
    slider_left_observed_rope_offset = Slider(
        ax=ax_left_observed_rope_offset,
        label='Left Observed Rope Offset',
        valmin=-50,
        valmax=50,
        valstep=1,
        valinit=0,
    )
    # Make a horizontal slider to control the frequency.
    ax_top_rope_length_offset = fig.add_axes([0.75, 0.25, 0.20, 0.03])
    slider_top_rope_length_offset = Slider(
        ax=ax_top_rope_length_offset,
        label='Top Rope Length Offset',
        valmin=constants.MIN_TOP_ROPE_LENGTH_OFFSET,
        valmax=constants.MAX_TOP_ROPE_LENGTH_OFFSET,
        valstep=constants.STEP_TOP_ROPE_LENGTH_OFFSET,
        valinit=constants.DEFAULT_TOP_ROPE_LENGTH_OFFSET,
    )

    # Make a horizontal slider to control the frequency.
    ax_angular_speed = fig.add_axes([0.75, 0.22, 0.20, 0.03])
    slider_angular_speed = Slider(
        ax=ax_angular_speed,
        label='Angular Speed [RPM]',
        valmin=constants.MIN_ANGULAR_SPEED,
        valmax=constants.MAX_ANGULAR_SPEED,
        valstep=constants.STEP_ANGULAR_SPEED,
        valinit=constants.DEFAULT_ANGULAR_SPEED
    )
    ax_hole_radius = fig.add_axes([0.75, 0.19, 0.20, 0.03])
    slider_hole_radius = Slider(
        ax=ax_hole_radius,
        label='Hole Radius [mm]',
        valmin=constants.MIN_HOLE_RADIUS,
        valmax=constants.MAX_HOLE_RADIUS,
        valstep=constants.STEP_HOLE_RADIUS,
        valinit=constants.DEFAULT_HOLE_RADIUS
    )

    ax_pole_height = fig.add_axes([0.75, 0.16, 0.20, 0.03])
    slider_pole_height = Slider(
        ax=ax_pole_height,
        label='Pole Height [mm]',
        valmin=constants.MIN_POLE_HEIGHT,
        valmax=constants.MAX_POLE_HEIGHT,
        valstep=constants.STEP_POLE_HEIGHT,
        valinit=constants.DEFAULT_POLE_HEIGHT,
    )

    ax_rope_length = fig.add_axes([0.75, 0.13, 0.20, 0.03])
    slider_rope_length = Slider(
        ax=ax_rope_length,
        label='Rope Length [mm]',
        valmin=constants.MIN_ROPE_LENGTH,
        valmax=constants.MAX_ROPE_LENGTH,
        valstep=constants.STEP_ROPE_LENGTH,
        valinit=constants.DEFAULT_ROPE_LENGTH,
    )
    
    def update_pole_height(pole_height):
        rope_length = slider_rope_length.val
        if pole_height >= rope_length:
            slider_pole_height.eventson = False
            slider_pole_height.set_val(rope_length - constants.STEP_ROPE_LENGTH)
            fig.canvas.draw()
            slider_pole_height.eventson = True

    def update_rope_length(rope_length):
        pole_height = slider_pole_height.val
        if  pole_height >= rope_length:
            slider_rope_length.eventson = False
            slider_rope_length.set_val(pole_height +constants.STEP_POLE_HEIGHT)
            fig.canvas.draw()
            slider_rope_length.eventson = True

    slider_pole_height.on_changed(update_pole_height)

    slider_rope_length.on_changed(update_rope_length)

    ax_pixel_pitch_vertical = fig.add_axes([0.75, 0.1, 0.20, 0.03])
    slider_pixel_pitch_vertical = Slider(
        ax=ax_pixel_pitch_vertical,
        label='Pixel Pitch Vertical [mm]',
        valmin=constants.MIN_PIXEL_PITCH,
        valmax=constants.MAX_PIXEL_PITCH,
        valstep=0.1, #constants.STEP_PIXEL_PITCH,
        valinit=constants.DEFAULT_PIXEL_PITCH,
    )
    
    ax_pixel_pitch_horizontal = fig.add_axes([0.75, 0.07, 0.20, 0.03])
    slider_pixel_pitch_horizontal = Slider(
        ax=ax_pixel_pitch_horizontal,
        label='Pixel Pitch Horizontal [mm]',
        valmin=constants.MIN_PIXEL_PITCH,
        valmax=constants.MAX_PIXEL_PITCH,
        valstep=0.1, #constants.STEP_PIXEL_PITCH,
        valinit=constants.DEFAULT_PIXEL_PITCH,
    )

    ax_pixel_mass  = fig.add_axes([0.75, 0.04, 0.20, 0.03])
    slider_pixel_mass = Slider(
        ax=ax_pixel_mass,
        label='Pixel Mass [gm/m]',
        valmin=constants.MIN_STRIP_MASS,
        valmax=constants.MAX_STRIP_MASS,
        valstep=constants.STEP_STRIP_MASS,
        valinit=constants.DEFAULT_STRIP_MASS
    )

    ax_solve_button = fig.add_axes([0.75, 0.01, 0.20, 0.03])
    button_solve = Button(
        ax=ax_solve_button, 
        label='Solve')
    def solve(event):
        W =  slider_angular_speed.val
        S =  slider_rope_length.val
        L =  slider_pole_height.val
        O =  slider_hole_radius.val
        Pv = slider_pixel_pitch_vertical.val
        Ph = slider_pixel_pitch_horizontal.val
        D = slider_pixel_mass.val/1000.0
        T = slider_top_rope_length_offset.val
        ObsLeftOffset = slider_left_observed_rope_offset.val
        
        if enable_3D:
            pixels_x, pixels_y, pixels_z,max_diameter, max_vertical_resolution, max_horizontal_resolution, central_tension, extremity_tension = troposkein3D(W, S, L, O, D, Pv,Ph, T, verbose=True)
            if image_path is not None:
                if poly:
                    image_bgr = cv2.imread(image_path)
                    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
                    image_pixels = []
                    image_start_v = image_rgb.shape[0]/2.0 - max_vertical_resolution/2.0
                    image_start_h = image_rgb.shape[1]/2.0 - max_horizontal_resolution/2.0
                    unique_points = np.unique(np.column_stack((pixels_x, pixels_y, pixels_z)), axis=1)
                    pixels_x = unique_points[:,0]
                    pixels_y = unique_points[:,1]
                    pixels_z = unique_points[:,2]
                    nb_rows = max_vertical_resolution
                    nb_cols = int(len(pixels_x)//max_vertical_resolution)
                    triangles = np.array([], dtype=int)
                    for i in tqdm(range(len(pixels_x))):
                        col = i // max_vertical_resolution
                        row = i % max_vertical_resolution
                        if row < nb_rows - 1 and col < nb_cols :
                            top_left = i % len(pixels_x)
                            top_right = (i + nb_rows) % len(pixels_x)
                            bottom_left = (i + 1) % len(pixels_x)
                            bottom_right = (i + nb_rows + 1) % len(pixels_x)
                            triangles = np.append(triangles, [top_left, bottom_left, top_right])
                            triangles = np.append(triangles, [bottom_left, bottom_right, top_right])
                            if row >= 0 and row < image_rgb.shape[0] and col >= 0 and col < image_rgb.shape[1]:
                                r, g, b = image_rgb[int(image_start_v + row), int(image_start_h + col)]
                                image_pixels.append(rgb2hex([r/255.0, g/255.0, b/255.0, 1.0], keep_alpha=True))
                                image_pixels.append(rgb2hex([r/255.0, g/255.0, b/255.0, 1.0], keep_alpha=True))
                                # image_pixels = np.append(image_pixels, [r/255.0, g/255.0, b/255.0, 1.0])
                                # image_pixels = np.append(image_pixels, [r/255.0, g/255.0, b/255.0, 1.0])
                            else:
                                image_pixels.append(rgb2hex([0.0, 0.0, 0.0, 0.0], keep_alpha=True))
                                image_pixels.append(rgb2hex([0.0, 0.0, 0.0, 0.0], keep_alpha=True))
                                # image_pixels = np.append(image_pixels, [0.0, 0.0, 0.0, 0.0])
                                # image_pixels = np.append(image_pixels, [0.0, 0.0, 0.0, 0.0])
                    triangles = triangles.reshape(-1, 3)
                    ax.clear()
                    scatter_surface = ax.plot_trisurf(pixels_x, pixels_y, pixels_z, triangles=triangles, cmap=None, linewidth=0.2)
                    scatter_surface.set_color(image_pixels)
                else:
                    image_bgr = cv2.imread(image_path)
                    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
                    image_pixels = []
                    image_start_v = image_rgb.shape[0]/2.0 - max_vertical_resolution/2.0
                    image_start_h = image_rgb.shape[1]/2.0 - max_horizontal_resolution/2.0
                    for h in range(max_horizontal_resolution):
                        for v in range(max_vertical_resolution):
                            if v >= 0 and v < image_rgb.shape[0] and h >= 0 and h < image_rgb.shape[1]:
                                r, g, b = image_rgb[int(image_start_v + v), int(image_start_h + h)]
                                image_pixels.append(rgb2hex([r/255.0, g/255.0, b/255.0, 1.0], keep_alpha=True))
                            else:
                                image_pixels.append(rgb2hex([0.0, 0.0, 0.0, 0.0], keep_alpha=True))
                    #scatter_surface = ax.scatter(pixels_x, pixels_y, pixels_z, s=1) #, c=image_pixels
                    scatter_surface.set_color("red")
                    scatter_surface._offsets3d = pixels_x, pixels_y, pixels_z
                    scatter_surface.set_color(image_pixels)
                # scatter_surface.set_color("red")
                # scatter_surface._offsets3d = pixels_x, pixels_y, pixels_z
                # scatter_surface.set_color(image_pixels)
            else:
                scatter_surface._offsets3d = pixels_x, pixels_y, pixels_z
                
            tropos_height = (max_diameter - 2.0 * O)/2.0
            table_vals = [
                ['%0.1f mm'%tropos_height],
                ['%0.1f mm'%max_diameter],
                ['%0.1f px'%max_vertical_resolution],
                ['%0.1f px'%max_horizontal_resolution],
                ['%0.3f N (%.4f kg)' %(central_tension, central_tension * 0.101972)],
                ['%0.3f N (%.4f kg)' %(extremity_tension, extremity_tension * 0.101972)]
            ]
        else:
            pixels_y, pixels_z,max_diameter, max_vertical_resolution, max_horizontal_resolution, central_tension, extremity_tension = troposkein2D(W, S, L, O, D, Pv, Ph, T, verbose=True)
            
            tropos_height = (max_diameter - 2.0 * O)/2.0
            scatter_surface.set_data(pixels_y, pixels_z)
            table_vals = [
                ['%0.1f mm'%tropos_height],
                ['%0.1f mm'%max_diameter],
                ['%0.1f px'%max_vertical_resolution],
                ['%0.1f px'%max_horizontal_resolution],
                ['%0.3f N (%.4f kg)' %(central_tension, central_tension * 0.101972)],
                ['%0.3f N (%.4f kg)' %(extremity_tension, extremity_tension * 0.101972)],
            ]

            if df_observation is not None:
                xObserved = df_observation.x.to_numpy() + constants.DEFAULT_HOLE_RADIUS + ObsLeftOffset
                yObserved = df_observation.y.to_numpy()
                interp_observed_func = interp1d(yObserved, xObserved,  fill_value="extrapolate")
                yNewObserved = pixels_z
                xNewObserved = interp_observed_func(yNewObserved)
                stackPixels = np.column_stack((pixels_y, pixels_z))
                stackObserved = np.column_stack((xNewObserved, yNewObserved))
                polygonObserved = np.r_[stackPixels, stackObserved[::-1]]
                fillPolygonObservedArea = area(polygonObserved)
                table_vals.append(['%0.3f mm2' %(np.sqrt(fillPolygonObservedArea))])

                fill_polygon_observed.set_xy(polygonObserved)
                scatter_observed.set_data(xNewObserved, yNewObserved)
                #scatter_observed_raw.set_data(xObserved, yObserved)


        for (row, col), cell in tab.get_celld().items():
            if row > 0 and col > -1:
                cell.get_text().set_text(table_vals[row-1][col])
        # self.ind += 1
        # i = self.ind % len(freqs)
        # ydata = np.sin(2*np.pi*freqs[i]*t)
        # l.set_ydata(ydata)
        # plt.draw()

    button_solve.on_clicked(solve)
    
    ax_solve_all_button = fig.add_axes([0.55, 0.01, 0.20, 0.03])
    button_solve_all = Button(
        ax=ax_solve_all_button, 
        label='Solve all')
    button_solve_all.on_clicked(solve_all)

    ax_save_button = fig.add_axes([0.35, 0.01, 0.20, 0.03])
    button_save = Button(
        ax=ax_save_button, 
        label='Save')
    def save(event):
        png_image_dir = "figures"
        # create a image name made of the parameters
        S =  slider_rope_length.val
        L =  slider_pole_height.val
        png_image_name = 'troposkein_H%d_L%d.png' %(L, S)
        # save figure to high-quality png file with transparent background
        fig.savefig(os.path.join(png_image_dir, png_image_name), dpi=300, transparent=True)

    button_save.on_clicked(save)
    # my_updater.add_ax(ax2, ['size'])  ##scatter plot, only marker size
    # my_updater.add_ax(ax3, ['alpha']) ##line plot, only alpha
    # my_updater.add_ax(ax4, ['size', 'alpha']) ##scatter plot, marker size and alpha
    
    # manager = plt.get_current_fig_manager()
    # manager.full_screen_toggle()
    plt.show()