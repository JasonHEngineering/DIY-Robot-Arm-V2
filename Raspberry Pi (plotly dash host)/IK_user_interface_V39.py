# -*- coding: utf-8 -*-
"""

V31 Baseline for Project Hayley V2 
V32 New linkages for Project Hayley V3
V33 Add dash page to process IK solved points and path from CSV
V34 Tidy up the HTML portion, further work on added dash page for IK path plan
V39 Update to handle CSV file - auto locate theta columns  

"""

import dash
from dash import dcc
from dash import html
import plotly.express as px
import plotly.graph_objs as go
import pandas as pd
import numpy as np
import smbus2 as smbus
import struct
import time
import math
from skspatial.objects import Plane, Point, Vector
import base64
import io
from dash import dash_table
import subprocess

# device address
arduino_IOT = 0x04

# initialize serial commumication
bus = smbus.SMBus(1)
time.sleep(1)

angle_slider_resolution = 0.1 # degrees


gear_reduction = [-30, -30, 30, -30, 30, 30] #-ve sign to flip direction of physical motor
steps_per_rotation = [200, 200, 200, 200, 200, 200]
micro_steps = [4, 4, 4, 4, 4, 4]

hinge_length = 14 # for illustrating the axis hinge in 3d view
max_iter = 500 # iterations allow for IK CCD
err_min = 2.5 # acceptable error in mm in inverse kinematics calculation

figure_axis_limit = 500

# config file to limit the max min angles of the axis
config_df = pd.read_csv('config - project Hayley V3.csv')
lower_limit = list(config_df["Absolute low angle limit"])
upper_limit = list(config_df["Absolute high angle limit"])

start_angles_list = list(config_df["Start angle"])
angles_list = start_angles_list.copy()

list_of_thetas = [0] * 32 # To be updated

index_of_angles = [2, 9, 12, 17, 23, 29] #hinge for project Hayley v3
#index_of_angles = [2, 9, 11, 16, 22, 28] #hinge 
index_of_hinges = index_of_angles.copy()

angle_max = list_of_thetas.copy()
angle_min = list_of_thetas.copy()
list_of_blockers = list_of_thetas.copy()

# [link number, theta about Z, alpha about X, d in Z direction, a or r in X direction]
# Link Length Parameter
# this is Project Hayley V3 linkage
local_linkage_data = [
    [0,0,0,0,0], #0
    [1,0,0,54,0], #1 
    [1,0,0,0,0], #theta_1 - Axis1, anti-clockwise with positive theta #2
    [2,0,-90,0,0], #3
    [2,0,0,35,0], #4 
    [2,0,90,0,0], #5
    [2,0,0,35.5,0], #6
    [2,0,90,0,0], #7
    [2,90,0,0,0], #8
    [2,0,0,0,0], #theta_2 - Axis2, down with positive theta #9
    [2,0,-180,0,0], #10
    [3,0,0,0,60], #11    
    [3,0,0,0,0], #theta_3 - Axis3, down with positive theta #12
    [4,0,0,0,33],     #13
    [4,0,0,-34.5,0],  # from here, switch between X and Z axis #14
    [4,90,0,0,0],     #15
    [4,0,90,0,0],     #16
    [4,0,0,0,0], #theta_4 - Axis4, anti-clockwise from top view with positive theta  #17
    [5,0,0,54,0], #18
    [5,0,-90,0,0],     #19
    [5,-90,0,0,0],     #20
    [5,0,0,32,0], #21
    [5,0,0,0,29],  # from here, switch between X and Z axis #22
    [5,0,0,0,0], #theta_5 - Axis5, down with positive theta #23
    [6,0,0,0,47],#24
    [6,0,0,-32,0], #25
    [6,0,0,0,25], #26
    [6,90,0,0,0],     #27
    [6,0,90,0,0],     #28    
    [6,0,0,0,0], #theta_6 - Axis6, anti-clockwise with positive theta #29
    [7,0,0,22,0], # dummy 20 mm extension #30
    [7,0,-90,0,0], # dummy 20 mm extension #31
    [7,-90,0,0,0], # dummy 20 mm extension #32
    ]


list_of_thetas = [0] * len(local_linkage_data)
angle_max = list_of_thetas.copy()
angle_min = list_of_thetas.copy()
list_of_blockers = list_of_thetas.copy()

for i, value in enumerate(index_of_angles):
    angle_max[value] = upper_limit[i]
    angle_min[value] = lower_limit[i]
    list_of_blockers[value] = 1
    

# special provision for the blockers to have theta to rotate the coodinate frame that is not due to hinge i.e theta due to mechanical link
for i, linkage in enumerate(local_linkage_data):
    if linkage[1] != 0:
        list_of_blockers[i] = 2
        list_of_thetas[i] = local_linkage_data[i][1]
        


def check_list(input_list):
    lower_exceed_count = 0
    upper_exceed_count = 0
    for i in range(len(input_list)):
        if input_list[i] < lower_limit[i]: lower_exceed_count=lower_exceed_count+1
        if input_list[i] > upper_limit[i]: upper_exceed_count=upper_exceed_count+1
    
    if lower_exceed_count>0 or upper_exceed_count>0: 
        return_string = "Angles out of range"
    else:
        return_string = "Angles in range"        

    return return_string


def DH_matrix(theta, alpha, delta, rho):

    transient_matrix = np.eye(4)
    
    # Handle DH parameters, row-by-row, left-to-right
    transient_matrix[0,0]=np.cos(theta/180*np.pi)
    transient_matrix[0,1]=-np.sin(theta/180*np.pi)    
    transient_matrix[0,2]=0    
    transient_matrix[0,3]=rho    

    transient_matrix[1,0]=np.sin(theta/180*np.pi)*np.cos(alpha/180*np.pi)
    transient_matrix[1,1]=np.cos(theta/180*np.pi)*np.cos(alpha/180*np.pi)  
    transient_matrix[1,2]=-np.sin(alpha/180*np.pi)     
    transient_matrix[1,3]=-np.sin(alpha/180*np.pi) * delta    

    transient_matrix[2,0]=np.sin(theta/180*np.pi)*np.sin(alpha/180*np.pi)
    transient_matrix[2,1]=np.cos(theta/180*np.pi)*np.sin(alpha/180*np.pi)  
    transient_matrix[2,2]=np.cos(alpha/180*np.pi)     
    transient_matrix[2,3]=np.cos(alpha/180*np.pi) * delta   

    return transient_matrix

def get_closest_coordinate_params(quadrant_coordinates_df):
    
    closest_coordinate = quadrant_coordinates_df[quadrant_coordinates_df['distance'] == quadrant_coordinates_df['distance'].min()]

    return(closest_coordinate)


def input_linkage_angles(list_of_thetas):
    
    for i in range(len(list_of_thetas)):
        # if list_of_blockers[i] == 2: # This is for special provision at axis 5
        #     list_of_thetas[i] = 90
        local_linkage_data[i][1] = list_of_thetas[i]

    array_matrix = []
    transformation_matrix = None
    
    for i, linkage in enumerate(local_linkage_data):
        
        # Rotations first
        transient_rotation = DH_matrix(linkage[1], linkage[2], 0, 0)
        
        if transformation_matrix is None:
            transformation_matrix = transient_rotation
        else:
            transformation_matrix = np.matmul(transformation_matrix, transient_rotation)
        
        # then the translations
        transient_translation = DH_matrix(0, 0, linkage[3], linkage[4])
        
        if transformation_matrix is None:
            transformation_matrix = transient_translation
        else:
            transformation_matrix = np.matmul(transformation_matrix, transient_translation)
        
        array_matrix.append(transformation_matrix)

    return(array_matrix)


def sqrt_sum_aquare(input_list):
    sum_square = 0
    for value in input_list:
        sum_square += value*value
    return(math.sqrt(sum_square))

def Inverse_Kinematics_CCD(target):
    solved = False
    err_end_to_target = math.inf
    minimum_error = math.inf
    
    for loop in range(max_iter):
        for i in range(len(local_linkage_data)-1, -1, -1):
            
            if list_of_blockers[i] != 0: # == 1 or 2 

                P = input_linkage_angles(list_of_thetas) # forward kinematics 
                end_to_target = target - P[-1][:3, 3]
                err_end_to_target = sqrt_sum_aquare(end_to_target)
                #error_list.append([loop, err_end_to_target])
                
                # record the angles of the best minimal error so far; yes the error can increase in further iterations
                if err_end_to_target < minimum_error:
                    minimum_error = err_end_to_target
                    least_error_angles = list_of_thetas.copy()                  
                
                if err_end_to_target < err_min:
                    solved = True
                else:
                    
                    if list_of_blockers[i] != 2:
                    
                        # Calculate distance between i-joint position to end effector position
                        # P[i] is position of current joint
                        # P[-1] is position of end effector
                        
                        # reviewed and change code here to improve since normal vector is always Z-axis if theta is always used as rotation
                        # use the DH array matrix, because in the top-left 3x3 sub-matrix, it already contains the vectors 
                        # for all 3 axis, top-row = X axis, middle = Y axis and last row = Z axis 
                        # this is with great hint from chatGPT with key question
                        # "find z-axis vector from denavit hartenberg matrix"
                        # attempt this update from V19
    
                        # find normal of rotation plane, aka hinge axis (hinge is always normal to rotation plane)
                        normal_vector = list(P[i][2, :3])
                        plane = Plane(point=P[i][:3, 3], normal=normal_vector)
                        
                        # find projection of tgt onto rotation plane
                        # https://scikit-spatial.readthedocs.io/en/stable/gallery/projection/plot_point_plane.html
                        target_point_projected = plane.project_point(target)
                        end_point_projected = plane.project_point(P[-1][:3, 3])
                        
                        # find angle between projected tgt and cur_to_end
                        cur_to_end_projected = end_point_projected - P[i][:3, 3]
                        cur_to_target_projected = target_point_projected - P[i][:3, 3]
    
                        # end_target_mag = |a||b|    
                        cur_to_end_projected_mag = sqrt_sum_aquare(cur_to_end_projected)
                        cur_to_target_projected_mag = sqrt_sum_aquare(cur_to_target_projected)
                        end_target_mag = cur_to_end_projected_mag * cur_to_target_projected_mag
    
                        # if the 2 vectors current-effector and current-target is already very close 
                        if end_target_mag <= 0.0001:    
                            cos_rot_ang = 1
                            sin_rot_ang = 0
                        else:
                            # dot product rule - https://en.wikipedia.org/wiki/Dot_product
                            # To solve for angle magnitude between 2 vectors
                            # dot product of two Euclidean vectors a and b
                            # a.b = |a||b|cos(lambda)
                            # cos_rot_ang = cos(lambda) = a.b / |a||b|
                            cos_rot_ang = (cur_to_end_projected[0] * cur_to_target_projected[0] + cur_to_end_projected[1] * cur_to_target_projected[1] + cur_to_end_projected[2] * cur_to_target_projected[2]) / end_target_mag
                            
                            # cross product rule - https://en.wikipedia.org/wiki/Cross_product
                            # https://www.mathsisfun.com/algebra/vectors-cross-product.html
                            # cross product of two Euclidean vectors a and b
                            # a X b = |a||b|sin(lambda)
                            # sin_rot_ang = sin(lambda) = [a X b] / |a||b|
                            # To solve for direction of angle A->B or B->A
                            # for theta rotation (about Z axis) in right hand rule, keep using [0] and [1] for finding Z direction
                            # cross product of 3d vectors has i, j, k components
                            # after we do the projections onto the plane level, we will focus on the k component                      
                            sin_rot_ang = (cur_to_end_projected[0] * cur_to_target_projected[1] - cur_to_end_projected[1] * cur_to_target_projected[0]) / end_target_mag   
                            
                        rot_ang = math.acos(max(-1, min(1,cos_rot_ang)))
        
                        if sin_rot_ang < 0.0:
                            rot_ang = -rot_ang
        
                        # Update current joint angle values
                        list_of_thetas[i] = list_of_thetas[i] + (rot_ang * 180 / math.pi)
                        list_of_thetas[i] = (list_of_thetas[i] + 180) % 360 - 180
    
                        # clamp angle
                        if list_of_thetas[i] > angle_max[i]: list_of_thetas[i] = angle_max[i]
                        if list_of_thetas[i] < angle_min[i]: list_of_thetas[i] = angle_min[i]
                        
                    elif list_of_blockers[i] == 2:  
                        #list_of_thetas[i] = 90
                        # there was a bug here befoew where the blockers force it to only positive 90 deg
                        # now I have update to adopt whatever linkage data is needed e.g. -90 deg aka 270 deg
                        list_of_thetas[i] = local_linkage_data[i][1] 

        if solved:
            break

    if solved == False:
        for i in range(len(list_of_thetas)):
            list_of_thetas[i] = least_error_angles[i] #return least error
            err_end_to_target = minimum_error
            P = input_linkage_angles(list_of_thetas) # forward kinematics
            
    return P, list_of_thetas, err_end_to_target, solved, loop


def add_trace(array_of_transformation_matrix):
    
    traces = []
    
    x_coordinate = []
    y_coordinate = []
    z_coordinate = []
        
    for i, transformation_matrix in enumerate(array_of_transformation_matrix):
        
        x_coordinate.append(transformation_matrix[0,3])
        y_coordinate.append(transformation_matrix[1,3])
        z_coordinate.append(transformation_matrix[2,3])
               
        
        if len(x_coordinate)==2:

            if x_coordinate[1]!=x_coordinate[0] or y_coordinate[1]!=y_coordinate[0] or z_coordinate[1]!=z_coordinate[0]:
            
                # print("Difference found at " + str(i))
                traces.append(go.Scatter3d(x=x_coordinate, y=y_coordinate, z=z_coordinate,
                                            #opacity=0.9,
                                            mode='lines',
                                            
                                            marker=dict(
                                                size=2
                                                ),
                                            line=dict(
                                                #name = linkage[0],
                                                #color=next(colors),
                                                color=px.colors.qualitative.Plotly[local_linkage_data[i][0]],
                                                width=15
                                              )
                                            ))
            x_coordinate.pop(0)
            y_coordinate.pop(0)
            z_coordinate.pop(0)

    return traces

def writeNumbers(values):
    # sending an float to the arduino
    # consider changing d to f for float precision
    byteList = []
    for value in values:
        value = int(value)
        byteList += list(struct.pack('i', value))
    print(byteList)
    byteList.append(0)  # fails to send last byte over I2C, hence this needs to be added 
    try:
        bus.write_i2c_block_data(arduino_IOT, byteList[0], byteList[1:len(byteList)])#[0:11]) byteList[0], 
    except IOError:
        subprocess.call(['i2cdetect', '-y', '1'])
    time.sleep(1)

def recalibrate_stepper():
    
    converted = []

    value = int(8) 
    converted += list(struct.pack('i', value)) #note that 4 bytes is sent, with a dummy register 0x00 in front
    
    BytesToSend = converted
    
    print(BytesToSend)
    BytesToSend.append(0)  # fails to send last byte over I2C, hence this needs to be added
    
    try:
        bus.write_i2c_block_data(arduino_IOT, BytesToSend[0], BytesToSend[1:len(BytesToSend)])
    except IOError:
        subprocess.call(['i2cdetect', '-y', '1'])
    time.sleep(1)

app = dash.Dash(__name__, suppress_callback_exceptions=True, title='6 DOF Kinematics') 

app.layout = html.Div([
    
    dcc.Store(id='memory-output'),

    html.Div([   
        
    html.Div([   
        
        html.Div([
            
            html.Div([   
                html.Img(src=app.get_asset_url('Jason_H_Engineering_Logo_100_100_V1.png')),
            ], className='one columns'),
            
            html.Br(),
        ], className='row'), 
    
        html.Div([
        
                dcc.RadioItems(
                    id='angle_or_coordinate_radio',
                    options=[{'label': i, 'value': i} for i in ['Angles', 'Target Coordinate', 'Path Plan']],
                    #options=['Angles', 'Target Coordinate'],
                    value='Angles',
                    labelStyle={'display': 'inline-block'}
                ),
                ], className='row'),
        
   
    ], className='three columns'),

    ], className='row'),
    
    html.Div([ 
        html.Div(id='div-content'),
    ], className='row'),

    ])


def angles_to_steps(angular_list):

    step_list = [0]*len(angular_list)
        
    for i in range(len(angular_list)):
  
        step_list[i] = round((angular_list[i] * gear_reduction[i] * micro_steps[i] / (360/steps_per_rotation[i])),0)
  
    return step_list

def div_header(value):
    
    if value == "Angles":
        html_string = "Enter Target Angles"
    else:
        html_string = "Enter Target Coordinates"

    appended_layout = html.Div([

        html.Div([
            html.H4(html_string)
        ], className='row'),
        
    ], className='row'),
        
    return appended_layout
    
def div_body(axis):
    
    html_string = "Axis " + str(axis+1) + ":"
    id_string = "angle_" + str(axis+1)
    
    body_layout = html.Div([
        html.B(html_string,style={'display':'inline-block','margin-right':5}),
        dcc.Slider(lower_limit[axis], upper_limit[axis], angle_slider_resolution,
            id=id_string,
            marks={
                lower_limit[axis]: {'label': lower_limit[axis], 'style': {'color': 'grey'}},
                upper_limit[axis]: {'label': upper_limit[axis], 'style': {'color': 'grey'}}
            },
            value=start_angles_list[axis],
            updatemode='drag',
            tooltip={"placement": "bottom", "always_visible": True}
        ),
    ], className='row')
        
    return body_layout

def add_axis_to_trace(traces, colour_string, axis_start, axis_end):

    rotate_axis = [axis_start, axis_start, axis_end]
    
    for i in range(len(colour_string)):

        traces.append(go.Scatter3d(x=rotate_axis[0], y=rotate_axis[1], z=rotate_axis[2],
                                   #opacity=0.7,
                                   mode='lines',
                                   marker=dict(
                                       color=colour_string[i],
                                       size=12
                                       )
                                   ))
        
        rotate_axis.append(rotate_axis.pop(0))
    
    return traces

def DH_array_to_hinge_list(traces, array_matrix):
    
    # hinge_type = ["delta","rho","rho","rho","delta"]
    
    for j in index_of_hinges:
        transformation_matrix = array_matrix[j]

        transient_translation = DH_matrix(0, 0, hinge_length, 0)      
        positive_z_matrix = np.matmul(transformation_matrix, transient_translation)
        transient_translation = DH_matrix(0, 0, -hinge_length, 0)
        #negative_z_matrix = np.matmul(transformation_matrix, transient_translation)

        # grab the +/- coordinates into array
        x_hinge = np.array([array_matrix[j][0,3],positive_z_matrix[0,3]])
        y_hinge = np.array([array_matrix[j][1,3],positive_z_matrix[1,3]])
        z_hinge = np.array([array_matrix[j][2,3],positive_z_matrix[2,3]])
        

        traces.append(go.Scatter3d(x=x_hinge, y=y_hinge, z=z_hinge,
                                   #opacity=0.7,
                                   mode='lines',
                                   marker=dict(
                                       #symbol="arrow",
                                       color="black",
                                       size=20
                                       ),
                                   line=dict(
                                    #color='purple',
                                    width=20)
                                   ))        

    return(traces)

def append_target_to_trace(traces, target):
    
    # append target point
    traces.append(go.Scatter3d(x=[target[0]], y=[target[1]], z=[target[2]],
                               opacity=0.7,
                               mode='markers',
                               marker=dict(
                                   color='black',
                                   size=12
                                   )
                               ))
    
    return(traces)


def parse_data(contents, filename):
    content_type, content_string = contents.split(",")

    decoded = base64.b64decode(content_string)
    try:
        if "csv" in filename:
            # Assume that the user uploaded a CSV or TXT file
            # df = pd.read_csv(io.StringIO(decoded.decode("utf-8")))
            df = pd.read_csv(io.StringIO(decoded.decode("ISO-8859-1")))
        elif "xls" in filename:
            # Assume that the user uploaded an excel file
            df = pd.read_excel(io.BytesIO(decoded))
        elif "txt" or "tsv" in filename:
            # Assume that the user upl, delimiter = r'\s+'oaded an excel file
            df = pd.read_csv(io.StringIO(decoded.decode("utf-8")), delimiter=r"\s+")
    except Exception as e:
        print(e)
        return html.Div(["There was an error processing this file."])

    return df

#Div handler
@app.callback(dash.dependencies.Output('div-content', 'children'),
              [dash.dependencies.Input('angle_or_coordinate_radio', 'value')])
def display_page(value):

    if value == "Angles":
    
        internal_layout = []
            
        for axis in range(len(start_angles_list)):
            internal_layout.append(div_body(axis))
    
        div_layout = html.Div([

            html.Div([
                
            html.Div([
                
            html.H5('Forward Kinematics - Angles (°)'),
            html.Br(),
            html.Div(internal_layout, className='row'),

            html.Br(),
            html.Button('Run', id='btn-1',style={'width': '280px'}),
            html.Br(),
            html.Button('Recalibrate Steppers', id='btn-5',style={'width': '280px'}),
            html.Br(),
            html.Button('Get Stepper Position', id='btn-4',style={'width': '280px'}),
            html.Div(id='stepper-position'),
            html.Br(),

            html.Div([
                html.B(id='target_coordinates',style={'color': 'grey', 'fontSize': 16}),
            ], className='row'),
            html.Br(),
            html.Div([
                html.B(id='target_angles',style={'color': 'grey', 'fontSize': 16}),
            ], className='row'),
            
            ], className='three columns'),
            
            html.Div([
                
            html.Div([
                html.H4("3D View")
            ]),   
             
            dcc.Loading(
                id="loading_1",
                children=[html.Div([dcc.Graph(id='go_graph_1')], className='row')],
                type="circle",
            ),
               
            ], className='nine columns'),
            ], className='row'),
        
        ])
    
    if value == "Target Coordinate":
        
        debounce_parameter = False
        state = False
        
        div_layout = html.Div([
            
            html.Div([
                
            html.H5('Inverse Kinematics - Cyclic Coordinate Descent (mm)'),
            html.Br(),
            
            html.Div([
                html.B('X:',style={'display':'inline-block','margin-right':5}),
                dcc.Input(id="x_coordinate_input", type="number", placeholder="0", debounce=debounce_parameter, disabled=state, style={'display':'inline-block', 'width': '50%'})
            ]),
            
            html.Div([
                html.B('Y:',style={'display':'inline-block','margin-right':5}),
                dcc.Input(id="y_coordinate_input", type="number", placeholder="0", debounce=debounce_parameter, disabled=state, style={'display':'inline-block', 'width': '50%'})
            ]),
            
            html.Div([
                html.B('Z:',style={'display':'inline-block','margin-right':5}),
                dcc.Input(id="z_coordinate_input", type="number", placeholder="0", debounce=debounce_parameter, disabled=state, style={'display':'inline-block', 'width': '50%'})
            ]),

            html.Br(),
            html.Button('Run', id='btn-1',style={'width': '280px'}),
            html.Br(),
            html.Button('Recalibrate Steppers', id='btn-5',style={'width': '280px'}),
            html.Br(),
            html.Button('Get Stepper Position', id='btn-4',style={'width': '280px'}),
            html.Div(id='stepper-position'),
            html.Br(),

            html.Div([
                html.B(id='target_coordinates',style={'color': 'grey', 'fontSize': 16}),
            ], className='row'),
            html.Br(),
            html.Div([
                html.B(id='target_angles',style={'color': 'grey', 'fontSize': 16}),
            ], className='row'),
            
            ], className='three columns'),
                        
            html.Div([
                
            html.Div([
                html.H4("3D View")
            ]),   
             
            dcc.Loading(
                id="loading_2",
                children=[html.Div([dcc.Graph(id='go_graph_2')], className='row')],
                type="circle",
            ),
            
            ], className='nine columns'),
            
        ])

    if value == "Path Plan":
        
        div_layout = html.Div([
            
        html.Div([
        html.Div([
            
        html.H5('Select/drop IK solution file!'),
            dcc.Upload(
                id="upload_data_1",
                children=html.Div(["Drag and Drop or ", html.A("Select Files")]),
                style={
                    "width": "100%",
                    "height": "60px",
                    "lineHeight": "60px",
                    "borderWidth": "1px",
                    "borderStyle": "dashed",
                    "borderRadius": "5px",
                    "textAlign": "center",
                    #"margin": "10px",
                },
                # Allow multiple files to be uploaded
                multiple=False,
            ),

        #], style={'marginTop': 25, 'marginLeft': 50, 'marginRight': 50}),
        ], className='six columns'),
        ], className='row'),
        
        # html.Div([
            
        # ],className='row'),            
        
        
        html.Div([

            # html.Div([            
                
            #     html.Br(),
            #     html.Button('Run all sequence', id='btn-6',style={'width': '280px'}),
            #     html.Br(),
                
            #     html.H6('Or select a specific target:'),
            #        html.Div([
            #            dcc.Dropdown(
            #                id='IK_target_ID',
            #                #options=[{'label': x, 'value': x} for x in fig_names],
            #                value=None,
            #                placeholder="Select target number",
            #        )],style={"width": "60%"},),
            #     html.Br(),
                
            #     html.Button('Recalibrate Steppers', id='btn-5',style={'width': '280px'}),
            #     html.Br(),
            #     html.Button('Get Stepper Position', id='btn-4',style={'width': '280px'}),
            #     html.Div(id='stepper-position'),
            #     html.Br(),                
                
            #       ], className='three columns'),
            
            # reflect raw table
            html.Div([
                 html.Div(id="raw-data-1-upload"),
                  ], className='nine columns'),

        ],className='row'),


    ]),
        
    return div_layout


# file upload handler
@app.callback(
    dash.dependencies.Output("upload_data_1", "children"), 
    dash.dependencies.Output("raw-data-1-upload", "children"),
    [dash.dependencies.Input("upload_data_1", "contents"), 
     dash.dependencies.Input("upload_data_1", "filename"),
     ],
)

# for displaying raw table
def update_children_1(contents_1, filename_1):

    if contents_1:

        df_1 = parse_data(contents_1, filename_1)
        return_div = html.Div([filename_1])
        html_string_1 = "Raw file: " + str(filename_1)
        
        sequence_id = df_1["Rows"]

        table = html.Div([
                #html.Hr(),
                
                html.Div([        
                    
                    html.Div([
                    
                        dcc.RadioItems(
                            id='run_method_radio',
                            options=[{'label': i, 'value': i} for i in ['None', 'Cubic Polynomial', 'Quintic Polynomial']],
                            value='None',
                            labelStyle={'display': 'inline-block'}
                        ),
                        ]),
                    
                    html.Br(),
                    html.Button('Run all sequence', id='btn-6',style={'width': '280px'}),
                    html.Br(),
                    
                    html.H6('Or select a specific target:'),
                       html.Div([
                           dcc.Dropdown(
                               id='IK_target_ID',
                               options=[{'label': x, 'value': x} for x in sequence_id],
                               value=None,
                               placeholder="Select target number",
                       )],style={"width": "60%"},),
                    html.Br(),                    
                    html.Button('Run Sequence', id='btn-7',style={'width': '280px'}),
                    html.Br(),                    
                    html.Button('Recalibrate Steppers', id='btn-5',style={'width': '280px'}),
                    html.Br(),
                    html.Button('Get Stepper Position', id='btn-4',style={'width': '280px'}),
                    html.Div(id='stepper-position'),
                    html.Br(),                
                    
                      ], className='three columns'),

                html.Div([        
                
                html.Br(), 
                html.Div(html_string_1),
                dash_table.DataTable(
                    id="table",
                    data=df_1.to_dict("rows"),
                    columns=[{"name": i, "id": i} for i in df_1.columns],
                    style_table={'height': '600px', 'overflowY': 'auto', 'overflowX': 'auto'},
                    style_cell={'textAlign': 'left',
                                'font_size': '16px'},
                    style_data={
                        'whiteSpace': 'normal',
                        'height': 'auto',
                    },
                    fill_width=False
                ),
            ], className='nine columns'),
                
            html.Div(id='running_status'),
                
            ])

    else:
        return_div = html.Div(["Drag and Drop or ", html.A("Select Files")])
        table = None

    return return_div, table






# callback for running selected sequence from CSV file
@app.callback(
    #dash.dependencies.Output("table", "style_data_conditional"), #https://community.plotly.com/t/highlighting-selected-rows/49595/5
    dash.dependencies.Output(component_id='running_status', component_property='children'),    
    [dash.dependencies.Input('btn-6', 'n_clicks'),
    dash.dependencies.Input("upload_data_1", "contents"), 
    dash.dependencies.Input("upload_data_1", "filename"),
    ], prevent_initial_call=True)
def sequence_run(btn6, contents_1, filename_1):

    changed_id = [p['prop_id'] for p in dash.callback_context.triggered][0]
    
    if 'btn-6' in changed_id:
        
        sequence_df = parse_data(contents_1, filename_1)
        
        # find the column "theta1" and "time_stay" and "Solve Status"
        theta_start_column = sequence_df.columns.get_loc('theta1')
        time_stay_column = sequence_df.columns.get_loc('time_stay')
        solve_status_column = sequence_df.columns.get_loc('Solve Status')
        
        for rows in range(len(sequence_df.index)):
            
            angles_list = sequence_df.iloc[rows][theta_start_column:(theta_start_column+len(index_of_angles))]
            time_stay = sequence_df.iloc[rows][time_stay_column]
            solve_status = sequence_df.iloc[rows][solve_status_column]
            
            if solve_status == True:
                #time_stay = 0.5
                
                # print(angles_list)
                # print(type(time_stay))
                
                check_string = check_list(angles_list)
                
                print(angles_list)
                print(check_string)
                
                if check_string == "Angles in range" :
                                
                    stepper_list = []
                    for i in range(len(angles_list)):
                        stepper_list.append(angles_list[i]-start_angles_list[i])
    
                    steps = angles_to_steps(stepper_list)
                    writeNumbers(steps)
                    #if rows == 0: time.sleep(time_stay) # seconds
                    time.sleep(time_stay)
                    print(time_stay)
                    #return f'Running sequence {rows}'

            #if rows == (len(sequence_df.index)-1) :
                   
        return "Sequence run complete" 
                        
            #if check_string == "Angles out of range" :
                   
                #return "Angles out of range" 
            
            #time.sleep(time_stay) # seconds
            


# callback for running selected sequence from CSV file
@app.callback(
    dash.dependencies.Output("table", "style_data_conditional"), #https://community.plotly.com/t/highlighting-selected-rows/49595/5
    [dash.dependencies.Input('run_method_radio', 'value'),
      dash.dependencies.Input('btn-7', 'n_clicks'),
    dash.dependencies.Input('IK_target_ID', 'value'),
    dash.dependencies.Input("upload_data_1", "contents"), 
    dash.dependencies.Input("upload_data_1", "filename"),
    ])
def move_to_point(method_value, btn7, sequence_id_value, contents_1, filename_1):

    changed_id = [p['prop_id'] for p in dash.callback_context.triggered][0]
    
    if 'btn-7' in changed_id:
        
        df_1 = parse_data(contents_1, filename_1,)
        
        angles_list = df_1.iloc[sequence_id_value-1][9:15]
        angles_list = list(angles_list)
        #print(angles_list)
    
        if sequence_id_value:
                  
            check_string = check_list(angles_list)
            
            #print(check_string)
            
            if check_string == "Angles in range" :
                            
                stepper_list = []
                for i in range(len(angles_list)):
                    stepper_list.append(angles_list[i]-start_angles_list[i])
    
                steps = angles_to_steps(stepper_list)
                writeNumbers(steps)
                
                return [
                    {
                        "if": {"row_index": sequence_id_value-1}, "backgroundColor": "dodgerblue",}
                ]            
                        
            if check_string == "Angles out of range" :
                   
                return [
                    {
                        "if": {"row_index": sequence_id_value-1}, "backgroundColor": "tomato",}
                ]

@app.callback(
    #dash.dependencies.Output(component_id='calibrating-string', component_property='children'),
    dash.dependencies.Output(component_id='angle_1', component_property='value'),
    dash.dependencies.Output(component_id='angle_2', component_property='value'),
    dash.dependencies.Output(component_id='angle_3', component_property='value'),
    dash.dependencies.Output(component_id='angle_4', component_property='value'),
    dash.dependencies.Output(component_id='angle_5', component_property='value'),
    dash.dependencies.Output(component_id='angle_6', component_property='value'),
    [dash.dependencies.Input('btn-5', 'n_clicks'),
], prevent_initial_call=True)
def recalibrate_stepper_callback(btn5):
    
    recalibrate_stepper()
    #calibrating_string = "Recalibrating"

    #return f'Output: {calibrating_string}', start_angles_list[0], start_angles_list[1], start_angles_list[2], start_angles_list[3], start_angles_list[4]    
    return start_angles_list[0], start_angles_list[1], start_angles_list[2], start_angles_list[3], start_angles_list[4], start_angles_list[5]


@app.callback(
    dash.dependencies.Output(component_id='stepper-position', component_property='children'),
    [dash.dependencies.Input('btn-4', 'n_clicks'),
], prevent_initial_call=True)
def update_stpper_position_div(btn4):
    
    data = []
    the_struct = [0, 0, 0, 0, 0, 0, 0] # first 6 for 6 motors, int calibration status,  
    
    changed_id = [p['prop_id'] for p in dash.callback_context.triggered][0]
    
    if 'btn-4' in changed_id:

        # test reference from https://stackoverflow.com/questions/40196733/how-to-pass-arduino-struct-to-raspberry-pi-via-i2c
    #while initialization_flag == 0:
        try:
            data=bus.read_i2c_block_data(arduino_IOT,0x00,28) #read_i2c_block_data(i2c_addr, register, length, force=None)
            print("recieve from slave:")
            print(data)
            
            byte_data = bytearray(data) # have to convert to array of bytes
            print(byte_data)
            the_struct = struct.unpack('iiiiiii', byte_data)
            print("unpacked as:")
            print(the_struct)
                        
        except:
            print("remote i/o error")
            time.sleep(0.5)    
        return f'Output: {the_struct}'

# 3d chart handler for angles input (forward kinematics)
@app.callback(
    dash.dependencies.Output('go_graph_1', 'figure'),
    dash.dependencies.Output('target_coordinates', 'children'),
    [dash.dependencies.Input('angle_or_coordinate_radio', 'value'),
    dash.dependencies.Input('btn-1', 'n_clicks'),
    dash.dependencies.Input('angle_1', 'value'),
    dash.dependencies.Input('angle_2', 'value'),
    dash.dependencies.Input('angle_3', 'value'),
    dash.dependencies.Input('angle_4', 'value'),
    dash.dependencies.Input('angle_5', 'value'),
    dash.dependencies.Input('angle_6', 'value')
    ])
def update_3d_graph_angles(value, btn1, angle_1, angle_2, angle_3, angle_4, angle_5, angle_6):
    
    # method to check if button click, so that fig update only upon click
    # https://stackoverflow.com/questions/62671226/plotly-dash-how-to-reset-the-n-clicks-attribute-of-a-dash-html-button
    changed_id = [p['prop_id'] for p in dash.callback_context.triggered][0]
    
    max_range = 0
    min_range = 0
    colour_list = ['rgb(0,100,80)', 'rgb(0,176,246)','rgb(231,107,243)']
    # list_of_thetas = [0] * 26
    
    if value == "Angles":
        
        if 'btn-1' in changed_id:
                  
            angles_list = [angle_1, angle_2, angle_3, angle_4, angle_5, angle_6]
            
            check_string = check_list(angles_list)
            
            if check_string == "Angles in range" :
                
                for i, row in enumerate(index_of_angles):
                    list_of_thetas[row] = angles_list[i]

                array_matrix = input_linkage_angles(list_of_thetas)
                traces = add_trace(array_matrix)

                min_range = min(array_matrix[-1][:3,3])
                max_range = max(array_matrix[-1][:3,3])   
                
                if abs(min_range) > abs(max_range):
                    max_range = -min_range
                else:
                    min_range = -max_range

                # add X/Y/Z axis
                axis_start = np.array([0,0])
                axis_end = np.array([min_range,max_range])         
                traces = add_axis_to_trace(traces, colour_list, axis_start, axis_end) # add the 3 axis 
                
                traces = DH_array_to_hinge_list(traces, array_matrix) # add hinges
                
                fig_range = [min_range, max_range]
                
                end_x_coordinate = round(array_matrix[-1][0,3],1)
                end_y_coordinate = round(array_matrix[-1][1,3],1)
                end_z_coordinate = round(array_matrix[-1][2,3],1)
                
                fig = go.Figure(data=traces)
                
                camera = dict(
                    eye=dict(x=1, y=1, z=0.7)
                )
                
                fig.update_layout(
                    scene_camera=camera,
                    scene = dict(
                    xaxis = dict(nticks=4, range=fig_range, visible = False, showticklabels = False),
                      yaxis = dict(nticks=4, range=fig_range, visible = False, showticklabels = False),
                      zaxis = dict(nticks=4, range=fig_range, visible = False, showticklabels = False),
                      # annotations for the axis
                      annotations=[
                      dict(
                          showarrow=False,
                          x=max_range,
                          y=0,
                          z=0,
                          text="<b>X</b>",
                          xanchor="left",
                          xshift=10,
                          #opacity=0.7
                          font=dict(
                            color=colour_list[2],
                            size=18
                            )
                          ),
                      dict(
                          showarrow=False,
                          x=0,
                          y=max_range,
                          z=0,
                          text="<b>Y</b>",
                          xanchor="left",
                          xshift=10,
                          #opacity=0.7
                          font=dict(
                            color=colour_list[1],
                            size=18
                            )
                          ),
                      dict(
                          showarrow=False,
                          x=0,
                          y=0,
                          z=max_range,
                          text="<b>Z</b>",
                          xanchor="left",
                          xshift=10,
                          #opacity=0.7
                          font=dict(
                            color=colour_list[0],
                            size=18
                            )
                          ),
                      ]
                      
                      ),
                            showlegend=False,
                            height=1200,
                            #width=1200
                            #font=dict(size=8)
                            margin=dict(
                                b=0, #bottom margin 0px
                                l=0, #left margin 0px
                                r=0, #right margin 0px
                                t=0, #top margin 0px
                            )
                                    )
                
                stepper_list = []
                for i in range(len(angles_list)):
                    stepper_list.append(angles_list[i]-start_angles_list[i])

                steps = angles_to_steps(stepper_list)
            
                writeNumbers(steps)

                        
            if check_string == "Angles out of range" :
                
                fig = go.Figure().add_annotation(x=2, y=2,text="No Data to Display",font=dict(family="sans serif",size=25,color="crimson"),showarrow=False,yshift=10)
                end_x_coordinate = 0
                end_y_coordinate = 0
                end_z_coordinate = 0
                
            return fig, f'X = {end_x_coordinate} mm, Y = {end_y_coordinate} mm, Z = {end_z_coordinate} mm'#, f'Y = {end_y_coordinate}, ', f'Z = {end_z_coordinate}'#, report_back_layout

# 3d chart handler for cooridnates input (inverse kinematics)
@app.callback(
    dash.dependencies.Output('go_graph_2', 'figure'),
    dash.dependencies.Output('target_angles', 'children'),
    [dash.dependencies.Input('angle_or_coordinate_radio', 'value'),
    dash.dependencies.Input('btn-1', 'n_clicks'),
    dash.dependencies.Input('x_coordinate_input', 'value'),
    dash.dependencies.Input('y_coordinate_input', 'value'),
    dash.dependencies.Input('z_coordinate_input', 'value')
    ])
def update_3d_graph_coordinates(value, btn1, x_coordinate_input, y_coordinate_input, z_coordinate_input):
    
    # method to check if button click, so that fig update only upon click
    # https://stackoverflow.com/questions/62671226/plotly-dash-how-to-reset-the-n-clicks-attribute-of-a-dash-html-button
    changed_id = [p['prop_id'] for p in dash.callback_context.triggered][0]
    
    max_range = 0
    min_range = 0
    colour_list = ['rgb(0,100,80)', 'rgb(0,176,246)','rgb(231,107,243)']
    
    if value == "Target Coordinate":
        
        if 'btn-1' in changed_id:

            target = [x_coordinate_input, y_coordinate_input, z_coordinate_input]
            
            array_matrix, list_of_angles, err_end_to_target, solve_status, iterations = Inverse_Kinematics_CCD(target)

            min_range = min(array_matrix[-1][:3,3])
            max_range = max(array_matrix[-1][:3,3])   
            
            if abs(min_range) > abs(max_range):
                max_range = -min_range
            else:
                min_range = -max_range
                
            fig_range = [min_range, max_range]
            
            # add X/Y/Z axis
            axis_start = np.array([0,0])
            axis_end = np.array([-figure_axis_limit,figure_axis_limit])
            colour_list = ['rgb(0,100,80)', 'rgb(0,176,246)','rgb(231,107,243)']
        
            traces = add_trace(array_matrix)    
            traces = add_axis_to_trace(traces, colour_list, axis_start, axis_end) # add the 3 axis 
            traces = DH_array_to_hinge_list(traces, array_matrix) # add hinges
            traces = append_target_to_trace(traces, target) # add target point
        
            fig = go.Figure(data=traces)
        
            annotation_text = (
                "Target Coordinates: " + str(target) + 
                "<br>" + "Actual Coordinates: " + str(np.round(array_matrix[-1][:3, 3],1)) + 
                "<br>" + "Error (mm): " + str(round(err_end_to_target,3)) +
                "<br>" + "IK CCD Iterations: " + str(iterations+1) + " (max: " + str(max_iter) + ")" +
                "<br>"
                )
            
            annotation_angle = ""
            display_angle = ""
            
            for i,index in enumerate(index_of_angles):
                annotation_angle = annotation_angle + "Axis " + str(i+1) + ": " + str(round(list_of_angles[index],1)) + "°" + "<br>"
                display_angle = display_angle + "Axis " + str(i+1) + ": " + str(round(list_of_angles[index],1)) + "°" + ", "
                
            annotation_text = annotation_text + annotation_angle
            
            fig.add_annotation(text=annotation_text,
                          xref="paper", yref="paper",
                          x=1, y=0.9, 
                          align='left',
                          showarrow=False) 

            camera = dict(
                eye=dict(x=1, y=1, z=0.7)
            )
            
            fig.update_layout(
                scene_camera=camera,
                scene = dict(
                xaxis = dict(nticks=4, range=fig_range, visible = False, showticklabels = False),
                  yaxis = dict(nticks=4, range=fig_range, visible = False, showticklabels = False),
                  zaxis = dict(nticks=4, range=fig_range, visible = False, showticklabels = False),
                  # annotations for the axis
                  annotations=[
                  dict(
                      showarrow=False,
                      x=max_range,
                      y=0,
                      z=0,
                      text="<b>X</b>",
                      xanchor="left",
                      xshift=10,
                      #opacity=0.7
                      font=dict(
                        color=colour_list[2],
                        size=18
                        )
                      ),
                  dict(
                      showarrow=False,
                      x=0,
                      y=max_range,
                      z=0,
                      text="<b>Y</b>",
                      xanchor="left",
                      xshift=10,
                      #opacity=0.7
                      font=dict(
                        color=colour_list[1],
                        size=18
                        )
                      ),
                  dict(
                      showarrow=False,
                      x=0,
                      y=0,
                      z=max_range,
                      text="<b>Z</b>",
                      xanchor="left",
                      xshift=10,
                      #opacity=0.7
                      font=dict(
                        color=colour_list[0],
                        size=18
                        )
                      ),
                  ]
                  
                  ),
                        showlegend=False,
                        height=1200,
                        #width=1200
                        #font=dict(size=8)
                        margin=dict(
                            b=0, #bottom margin 0px
                            l=0, #left margin 0px
                            r=0, #right margin 0px
                            t=0, #top margin 0px
                        )
                                )
            
            
            stepper_list = []

            extracted_angles = [0]*len(index_of_angles)
            
            for i, row in enumerate(index_of_angles):
                extracted_angles[i] = list_of_angles[row]

            check_string = check_list(extracted_angles)
            
            if (check_string == "Angles in range") and (err_end_to_target < err_min):
                for i in range(len(extracted_angles)):
                    stepper_list.append(extracted_angles[i]-start_angles_list[i])
                
                steps = angles_to_steps(stepper_list)
                
                writeNumbers(steps)
            
            if err_end_to_target > err_min:
                
                display_angle = "CCD IK Solution not found or angles are out of range!"
                
            # return fig, f'X = {end_x_coordinate} mm, Y = {end_y_coordinate} mm, Z = {end_z_coordinate} mm'#, f'Y = {end_y_coordinate}, ', f'Z = {end_z_coordinate}'#, report_back_layout
            return fig, f'{display_angle}'#, f'Y = {end_y_coordinate}, ', f'Z = {end_z_coordinate}'#, report_back_layout


if __name__ == '__main__':
    app.run_server(host= '0.0.0.0',debug=False)
        