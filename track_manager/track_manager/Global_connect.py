import numpy as np
from scipy.special import comb
from Track import Track
import matplotlib.pyplot as plt
from Track import Track

# need Track Package for scad generation

class Traintrack:
    def __init__(self):
        pass

def curve_curvature(x, y):
    # Calculate first derivatives
    dx_dt = np.gradient(x)
    dy_dt = np.gradient(y)

    # Calculate second derivatives
    dx2_dt2 = np.gradient(dx_dt)
    dy2_dt2 = np.gradient(dy_dt)

    # Calculate curvature
    curvature = np.abs(dx_dt * dy2_dt2 - dy_dt * dx2_dt2) / (dx_dt**2 + dy_dt**2)**1.5

    return curvature

def bernstein_poly(i, n, t):
    return comb(n, i) * (t ** (n - i)) * ((1 - t) ** i)

def bezier_curve(control_points, num_points=100):
    n = len(control_points) - 1
    t_values = np.linspace(0, 1, num_points)
    curve_points = np.zeros((num_points, 2))

    for i, t in enumerate(t_values):
        curve_points[i] = np.sum([control_points[j] * bernstein_poly(j, n, t) for j in range(n + 1)], axis=0)

    return curve_points

def gen_train_curves(start_points,start_deg,end_points,end_deg,control_lenght=300,min_rad=100,draw_lines=False):
    # train track global generattion.
    # provide a list of start points and end points with direction information
    # generate mulitiple curves for user to generate final track

    correct_number = False
    in_range = True
    if len(start_points) == len(start_deg) == len(end_points) == len(end_deg):
        correct_number = True
    if not correct_number:
        raise ValueError("Input lengths are different.")
    else:
        total_sections = len(start_points)
        curves = []

        for n in range(total_sections):
            start_x = start_points[n,0]
            start_y = start_points[n,1]
            end_x = end_points[n,0]
            end_y = end_points[n,1]
            curve,control_p1,control_p2 = gen_one_path(start_x,start_y,start_deg[n],end_x,end_y,end_deg[n],control_lenght)
            curves.append(curve)
            x_values = np.flip(np.array(curve[:, 0]))
            # print(len(x_values))
            y_values = np.flip(np.array(curve[:, 1]))
            curvature_values = curve_curvature(x_values, y_values)
            min_radius = np.min(1/curvature_values)
            if min_radius<min_rad:
                print(min_radius)
                print("False: Unable to generate")
                in_range = False
                return in_range,curves


            else:
                print("Connected successfully")


            if draw_lines:
                plt.plot([start_x, control_p1[0]], [start_y, control_p1[1]], 'ro-', label='Line 1')
                plt.plot([control_p2[0], end_x], [control_p2[1], end_y], 'go-', label='Line 2')

                # Plot the intermediate points
                plt.plot(control_p1[0], control_p1[1], 'ro', label='Intermediate Point 1')
                plt.plot(control_p2[0], control_p2[1], 'go', label='Intermediate Point 2')

                plt.plot(curve[:, 0], curve[:, 1], 'y-', label='Bézier Curve')
            
        if draw_lines:
            # Plot the reference lines
            # Customize the plot
            plt.title('Spline Connecting Two Lines')
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.legend()
            plt.grid(True)
            plt.axis('equal') 
            plt.show()
    return in_range,curves



def gen_one_path(start_x, start_y, start_deg, end_x, end_y, end_deg, control_length):
    angle_radians1 = np.radians(start_deg)
    angle_radians2 = np.radians(end_deg)

    min_dis = np.sqrt((start_x - end_x)**2+(start_y - end_y)**2)
    if control_length>min_dis:
        control_length = min_dis

    control_x1 = start_x + control_length/2 * np.cos(angle_radians1)
    control_y1 = start_y + control_length/2 * np.sin(angle_radians1)
    control_x2 = end_x - control_length/2 * np.cos(angle_radians2)
    control_y2 = end_y - control_length/2 * np.sin(angle_radians2)

    line_start = np.array([start_x, start_y])
    line_end = np.array([end_x, end_y])

    control_p1 = np.array([control_x1, control_y1])
    control_p2 = np.array([control_x2, control_y2])

    x_data = np.array([line_start[0], control_p1[0],control_p2[0], line_end[0]])
    y_data = np.array([line_start[1], control_p1[1],control_p2[1], line_end[1]])

    t_value = 50

    control_points = np.column_stack((x_data, y_data))
    curve = np.array(bezier_curve(control_points,t_value))
    curve = np.flip(curve,0)
    return curve,control_p1,control_p2

def track_to_scad(curves, split_lenght, cross_shape,base_filename = 'part_arc',to_stl = False):
    num_tracks = len(curves)
    filenames = []
    for i in range(num_tracks):
        filename = f"{base_filename}_{i + 1}"
        
        
        curve_points = np.array(curves[i])
        curve_3d = np.c_[curve_points, np.zeros(curve_points.shape[0])]
        track = Track(curve_3d)
        track.split_by_length(split_lenght)
        files = track.save_scad_files(cross_shape,filename,to_stl=to_stl)
        filenames.append(files)
    return filenames

# function to generate train tracks
# this function connected to the tag generator and track generator which will generate the track with id

def gen_train_tracks(starts,start_degs,ends,end_degs,draw_lines=False,base_filename = 'part',to_stl = False):
    status, curves = gen_train_curves(starts,start_degs,ends,end_degs,draw_lines=draw_lines,min_rad=100)
    if status == False:
        print("error, unable to gen")
        return False,[]
    else:
        trackshape = [[0,0],[20,0],[20,10],[17,10],[17,7],[10,7],[10,10],[0,10],[-10,10],[-10,7],[-17,7],[-17,10],[-20,10],[-20,0]]

        filenames = track_to_scad(curves,200,trackshape,to_stl=to_stl)
        return True, filenames

# if __name__ == "__main__":
#     starts = np.array([[187.5,-17.5]])
#     ends = np.array([[322.1,-25.5]])
#     # starts = np.array([[200,0]])
#     # ends = np.array([[300,0]])
#     start_degs = np.array([0])
#     end_degs = np.array([-17.67])
#     # end_degs = np.array([0])

#     status, filenames = gen_train_tracks(starts,start_degs,ends,end_degs,draw_lines=True,to_stl=True)
#     print(filenames)

    # status, curves = gen_train_curves(starts,start_degs,ends,end_degs,draw_lines=True)
    # if status == False:
    #     print("error, unable to gen")
    # else:
    #     trackshape = [[0,0],[20,0],[20,10],[15,10],[15,7],[10,7],[10,10],[0,10],[-10,10],[-10,7],[-15,7],[-15,10],[-20,10],[-20,0]]

    #     track_to_scad(curves,200,trackshape)
