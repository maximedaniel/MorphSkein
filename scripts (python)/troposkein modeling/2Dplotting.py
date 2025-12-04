from matplotlib import pyplot as plt
import numpy as np
from troposkein import *
from matplotlib.font_manager import FontProperties
from matplotlib.widgets import Slider, Button
from matplotlib import cm

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



LED_METER_DENSITY = 144.0/1000.0 #leds/m
PIXEL_PITCH = 7 #cm

# hole width
MIN_W = 206 # 205.782cm
MAX_W = MIN_W * 2 #cm
STEP_W = PIXEL_PITCH #cm

MIN_VERTICAL_RESOLUTION = 30 # 21cm / 0.7cm
MAX_VERTICAL_RESOLUTION = MIN_VERTICAL_RESOLUTION + 86
 

# strip length
MIN_ROPE_LENGTH = MIN_VERTICAL_RESOLUTION * PIXEL_PITCH
MAX_ROPE_LENGTH = MAX_VERTICAL_RESOLUTION * PIXEL_PITCH
STEP_ROPE_LENGTH = PIXEL_PITCH

# strip height
MIN_POLE_HEIGHT = 195
MAX_POLE_HEIGHT = MIN_POLE_HEIGHT + 46 * PIXEL_PITCH
STEP_POLE_HEIGHT = PIXEL_PITCH

diff_tol = 1e-8
stop_tol = 1e-8
eps_tol = 1e-8

LED_BITS = 24
FPS = 24
max_vertical_resolution = 0
max_horizontal_resolution = 0
fitted_length = 0
fitted_height = 0
hole_diameter = MIN_W
max_diameter = 0
max_perimeter = 0
pixel_density = 0
pixel_pitch = 0

all_solve_done = False

def triangular_index (h_value, l_value):
    h_size =  (MAX_POLE_HEIGHT - MIN_POLE_HEIGHT)/STEP_POLE_HEIGHT + 1 # 3
    l_size =  (MAX_ROPE_LENGTH - MIN_ROPE_LENGTH)/STEP_ROPE_LENGTH + 1 # 61
    h_index = (h_value - MIN_POLE_HEIGHT)/STEP_POLE_HEIGHT #0 - 2
    l_index = (l_value - MIN_ROPE_LENGTH)/STEP_ROPE_LENGTH #0 - 60
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
     P = slider_pixel_pitch.val
     D = slider_pixel_mass.val/1000.0
     res = ""
     h_params = []
     l_params = []
     vertical_resolution_results = []
     horizontal_resolution_results = []
     total_resolution_results = []
     h_values = np.arange(MIN_POLE_HEIGHT, MAX_POLE_HEIGHT + 1, STEP_POLE_HEIGHT).astype(int)
     print(h_values)
     l_values = np.arange(MIN_ROPE_LENGTH, MAX_ROPE_LENGTH + 1, STEP_ROPE_LENGTH).astype(int)
     print(l_values)
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
            pixels_x, pixels_y, pixels_z,max_diameter, max_vertical_resolution, max_horizontal_resolution, central_tension, extremity_tension = troposkein3D(W, S, L, O, D, P, verbose=False)
            vertical_resolution_results.append(int(max_vertical_resolution))
            horizontal_resolution_results.append(int(max_horizontal_resolution))
            total_resolution_results.append(max_vertical_resolution * max_horizontal_resolution)
            triangle_index = triangular_index(h, l)
            #print("%d" %(triangle_index))
            if triangle_index <= prev_triangle_index:
                print("bad index: %d" %(triangle_index))
                exit()
            # else:
            #     print("horizontal_resolution_result: %d" %(horizontal_resolution_results[triangle_index]))
            print("[%d] (W@%dmm, H@%dmm, L@%dmm) -> %dpx" %(triangle_index, hole_diameter, h, l, max_horizontal_resolution))
            prev_triangle_index = triangle_index
        print("%d  -> %d" %(i, len(next_l_values)))
     print("Done.")
        
     max_vertical_resolution_results = max(vertical_resolution_results)
     max_horizontal_resolution_results = max(horizontal_resolution_results)
     max_total_resolution_results = max(total_resolution_results)

     horizontal_resolution_results_str = str(horizontal_resolution_results)
     #print(l_params)
     res += '#define DISPLAY_MAX_HORIZONTAL_RESOLUTION %d\n' %max_horizontal_resolution_results
     res += '#define DISPLAY_MAX_VERTICAL_RESOLUTION %d\n' %max_vertical_resolution_results
     res += '#define DISPLAY_MAX_NB_PIXELS %d\n' %max_total_resolution_results
     res += '#define MIN_H %d\n' %MIN_POLE_HEIGHT
     res += '#define MAX_H %d\n' %MAX_POLE_HEIGHT  
     res += '#define STEP_H %d\n' %STEP_POLE_HEIGHT
     res += '#define NB_H %d\n' %len(h_values)
     res += '#define MIN_L %d\n' %MIN_ROPE_LENGTH  
     res += '#define MAX_L %d\n' %MAX_ROPE_LENGTH  
     res += '#define STEP_L %d\n' %STEP_ROPE_LENGTH
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




if __name__ == '__main__':
    height_factors = np.arange(1.0, 2.51, 0.5)
    print('height_factors: %s' %height_factors)
    height_values = height_factors * MIN_POLE_HEIGHT
    print('height_values: %s' %height_values)
    length_factors = np.arange(1.0, 4.01, 0.5)
    print('length_factors: %s' %length_factors)
    length_values = length_factors * MIN_ROPE_LENGTH
    print('length_values: %s' %length_values)
    length_values = length_values[ length_values <= MAX_ROPE_LENGTH]
    print('length_values (clamped): %s' %length_values)
    length_values =  np.round(length_values / 7.0) * 7
    print('length_values (to pixel density): %s' %length_values)

    # H_L_init = MIN_ROPE_LENGTH/MIN_POLE_HEIGHT
    # H_L_ratios = np.arange(H_L_init, H_L_init * 4.51, 0.5)
    # print('H_L_ratios: %s' %H_L_ratios)

    # # print('length_factors: %s' %length_factors)
    # # length_values = length_factors * MIN_ROPE_LENGTH
    # # print('length_values: %s' %length_values)
    for height_value in height_values:
            filtered_length_values = length_values[ length_values > height_value]
            print('%f -> %s' %(height_value, filtered_length_values))
            vertical_resolution_delta = (filtered_length_values - MIN_ROPE_LENGTH)/PIXEL_PITCH
            print('%f -> %s' %(height_value,vertical_resolution_delta))
            rounded_vertical_resolution_delta = np.round(vertical_resolution_delta)
            print('%.2f -> %s' %(height_value,rounded_vertical_resolution_delta))
    #     print('%.2f -> %s' %(height_value,rounded_vertical_resolution_delta))

    my_updater = MarkerUpdater()
    init_angular_speed = 150
    init_hole_radius = 103
    init_pole_height = MIN_POLE_HEIGHT
    init_rope_length = MIN_ROPE_LENGTH
    init_pixel_pitch = 7
    init_pixel_mass = 22
    W=init_angular_speed
    S=init_rope_length
    L=init_pole_height
    O=init_hole_radius
    D=init_pixel_mass/1000.0
    P=init_pixel_pitch
    pixels_y, pixels_z, max_diameter, max_vertical_resolution, max_horizontal_resolution, central_tension, extremity_tension = troposkein2D(W, S, L, O, D, P, verbose=True)
    #defining the attributes
    col_labels = ['']
    col_widths = [0.25]
    row_labels = ['Max diameter','Vertical Resolution','Horizontal Resolution', 'Central Tension', 'Extremity Tension' ]
    table_vals = [
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
    #fig, axes = plt.subplots(111, projection='3d')#, figsize=(1,1))
    
    # ax = fig.add_subplot(111, projection='3d', proj_type = 'ortho')

    # ax.set_box_aspect((np.ptp(pixels_x), np.ptp(pixels_y), np.ptp(pixels_z)))
    # scatter_surface = ax.scatter(pixels_x, pixels_y, pixels_z, s=1)

    ax = fig.add_subplot(111)
    ax.set_xlim([0, O + MAX_ROPE_LENGTH/2.0 ])
    ax.set_ylim([0, MAX_POLE_HEIGHT])
    ax.set_aspect('equal')
    #ax.set_box_aspect((np.ptp(pixels_y), np.ptp(pixels_z)))
    scatter_surface,  = ax.plot(pixels_y, pixels_z, linewidth=2, markersize=2, marker='o', color='black',)


    #surf = ax.plot_trisurf(pixels_x, pixels_y, pixels_z, cmap=cm.jet, linewidth=0)


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
    ##setting up the updater
    my_updater.add_ax(ax, ['size'])  ##line plot, only marker size
    # Make a horizontal slider to control the frequency.
    ax_angular_speed = fig.add_axes([0.75, 0.19, 0.20, 0.03])
    slider_angular_speed = Slider(
        ax=ax_angular_speed,
        label='Angular Speed [RPM]',
        valmin=100,
        valmax=400,
        valstep=1,
        valinit=init_angular_speed,
    )
    ax_hole_radius = fig.add_axes([0.75,  0.16, 0.20, 0.03])
    slider_hole_radius = Slider(
        ax=ax_hole_radius,
        label='Hole Radius [mm]',
        valmin=init_hole_radius,
        valmax=init_hole_radius + 100,
        valstep=1,
        valinit=init_hole_radius,
    )

    ax_pole_height = fig.add_axes([0.75,  0.13, 0.20, 0.03])
    slider_pole_height = Slider(
        ax=ax_pole_height,
        label='Pole Height [mm]',
        valmin=MIN_POLE_HEIGHT,
        valmax=MAX_POLE_HEIGHT,
        valstep=STEP_POLE_HEIGHT,
        valinit=init_pole_height,
    )

    ax_rope_length = fig.add_axes([0.75, 0.1, 0.20, 0.03])
    slider_rope_length = Slider(
        ax=ax_rope_length,
        label='Rope Length [mm]',
        valmin=MIN_ROPE_LENGTH,
        valmax=MAX_ROPE_LENGTH,
        valstep=STEP_ROPE_LENGTH,
        valinit=init_rope_length,
    )
    
    def update_pole_height(pole_height):
        rope_length = slider_rope_length.val
        if pole_height >= rope_length:
            slider_pole_height.eventson = False
            slider_pole_height.set_val(rope_length - STEP_ROPE_LENGTH)
            fig.canvas.draw()
            slider_pole_height.eventson = True

    def update_rope_length(rope_length):
        pole_height = slider_pole_height.val
        if  pole_height >= rope_length:
            slider_rope_length.eventson = False
            slider_rope_length.set_val(pole_height + STEP_POLE_HEIGHT)
            fig.canvas.draw()
            slider_rope_length.eventson = True

    slider_pole_height.on_changed(update_pole_height)

    slider_rope_length.on_changed(update_rope_length)

    ax_pixel_pitch = fig.add_axes([0.75,  0.07, 0.20, 0.03])
    slider_pixel_pitch = Slider(
        ax=ax_pixel_pitch,
        label='Pixel Pitch [mm]',
        valmin=1,
        valmax=10,
        valstep=1,
        valinit=init_pixel_pitch,
    )

    ax_pixel_mass  = fig.add_axes([0.75, 0.04, 0.20, 0.03])
    slider_pixel_mass = Slider(
        ax=ax_pixel_mass,
        label='Pixel Mass [gm/m]',
        valmin=22,
        valmax=220,
        valstep=1,
        valinit=init_pixel_mass,
    )

    ax_solve_button = fig.add_axes([0.75,  0.01, 0.20, 0.03])
    button_solve = Button(
        ax=ax_solve_button, 
        label='Solve')
    def solve(event):
        W =  slider_angular_speed.val
        S =  slider_rope_length.val
        L =  slider_pole_height.val
        O =  slider_hole_radius.val
        P = slider_pixel_pitch.val
        D = slider_pixel_mass.val/1000.0
        pixels_y, pixels_z,max_diameter, max_vertical_resolution, max_horizontal_resolution, central_tension, extremity_tension = troposkein2D(W, S, L, O, D, P, verbose=True)
        table_vals = [
            ['%0.1f mm'%max_diameter],
            ['%0.1f px'%max_vertical_resolution],
            ['%0.1f px'%max_horizontal_resolution],
            ['%0.3f N (%.4f kg)' %(central_tension, central_tension * 0.101972)],
            ['%0.3f N (%.4f kg)' %(extremity_tension, extremity_tension * 0.101972)]
        ]
        
        scatter_surface.set_data(pixels_y, pixels_z)
        
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
    # my_updater.add_ax(ax2, ['size'])  ##scatter plot, only marker size
    # my_updater.add_ax(ax3, ['alpha']) ##line plot, only alpha
    # my_updater.add_ax(ax4, ['size', 'alpha']) ##scatter plot, marker size and alpha
    
    # manager = plt.get_current_fig_manager()
    # manager.full_screen_toggle()
    plt.show()