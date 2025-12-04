import numpy as np
import pandas as pd
from scipy import stats
import time
import matplotlib.pyplot as pyplot
from matplotlib import cm
from matplotlib import animation
from colour import Color
import seaborn as sns
import random
from scipy.spatial.transform import Rotation as R

NB_ROWS = 5
NB_COLUMNS = 6

class SpacePlotter:

    def rotateInQuaternion(self, position_matrix, quaternion):
        rotation_matrix =  R.from_quat(quaternion).as_matrix()
        rotated_position_matrix = np.dot(rotation_matrix, position_matrix)
        return rotated_position_matrix


    def rotateInDirection(self, position_matrix, direction, axis):
        estimated_rotation, rmsd = R.align_vectors([direction], [axis])
        rotation_matrix =  estimated_rotation.as_matrix()
        rotated_position_matrix = np.dot(rotation_matrix, position_matrix)
        return rotated_position_matrix

    def generateCylinder(self, center_x, center_y, center_z, radius, height_z, resolution):
        z = np.linspace(center_z - height_z/2.0, center_z + height_z/2.0, resolution)
        theta = np.linspace(0, 2*np.pi, resolution)
        theta_grid, z_grid=np.meshgrid(theta, z)
        x_grid = radius*np.cos(theta_grid) + center_x
        y_grid = radius*np.sin(theta_grid) + center_y
        return np.array([x_grid.flatten(),y_grid.flatten(),z_grid.flatten()])

    def generateSphere(self, center_x, center_y, center_z, radius, resolution):
        phi, theta = np.mgrid[0.0:np.pi:(resolution * 1j), 0.0:2.0*np.pi:(resolution * 1j)]
        x_grid = radius*np.sin(phi)*np.cos(theta) + center_x
        y_grid = radius*np.sin(phi)*np.sin(theta) + center_y
        z_grid = radius*np.cos(phi) + center_z
        return np.array([x_grid.flatten(),y_grid.flatten(),z_grid.flatten()])

    def __init__(self, df):
        self.resolution = 25
        self.df = df
        self.data = None
        self.i = 0
        self.N = self.df.shape[0]
        self.leftHandMesh = None
        self.leftArmMesh = None
        self.rightHandMesh = None
        self.rightArmMesh = None

        fig = pyplot.figure()
        self.ax = fig.add_subplot(111, projection='3d')
        # start_direction = np.array([0,0,0])
        # end_direction = np.array([0.3,0,0.3])
        # targetDirection = end_direction - start_direction
        # self.ax.plot([start_direction[0], end_direction[0]], [start_direction[1], end_direction[1]], [start_direction[2], end_direction[2]])
        # CylinderPosition = self.generateCylinder(center_x=0.2,center_y=0.2,radius=0.05,height_z=0.1,resolution=50) 
        # CylinderDirection = np.array([0,0,1])
        # CylinderPosition = self.rotateInDirection(CylinderPosition, targetDirection, CylinderDirection)
        # self.ax.scatter(CylinderPosition[0],CylinderPosition[1], CylinderPosition[2], marker='o')

        self.ax.set_xlim([-100,100])
        self.ax.set_ylim([-100,100])
        self.ax.set_zlim([-100,100])
        self.ax.set_aspect("auto")
            
        anim = animation.FuncAnimation(fig, self.animate, init_func=self.initialize, interval=50, frames=self.N, repeat=False)
        pyplot.show()
    
    def update(self):
        if self.i < self.N:
            self.leftHandMesh = None
            self.leftArmMesh = None
            self.rightHandMesh = None
            self.rightArmMesh = None
            self.data = self.df.iloc[self.i]
            self.i += 1
            #print("%s/%s" %(self.i, self.N))
            if not np.isnan(self.data['LEFT_HAND_POSITION_X']):
                # LEFT HAND MESH
                self.leftHandPos = self.data[['LEFT_HAND_POSITION_X','LEFT_HAND_POSITION_Y', 'LEFT_HAND_POSITION_Z']].values
                self.leftHandRadius = self.data['LEFT_HAND_RADIUS']
                self.leftHandMesh = self.generateSphere(
                    center_x = self.leftHandPos[0],
                    center_y = self.leftHandPos[1],
                    center_z = self.leftHandPos[2],
                    radius  = self.leftHandRadius * 2.0,
                    resolution=self.resolution) 
                # LEFT ARM MESH
                self.leftArmPos = self.data[['LEFT_ARM_POSITION_X','LEFT_ARM_POSITION_Y', 'LEFT_ARM_POSITION_Z']].values
                self.leftArmDir = self.leftHandPos - self.leftArmPos
                leftMagnitude = np.sqrt(self.leftArmDir.dot( self.leftArmDir))
                self.leftArmPos -= self.leftArmDir * ((self.leftHandRadius*2.0)/leftMagnitude)
                self.leftArmRadius = self.data['LEFT_ARM_RADIUS']
                self.leftArmHeight = self.data['LEFT_ARM_HEIGHT']
                self.leftArmRot = self.data[['LEFT_ARM_QUAT_A','LEFT_ARM_QUAT_B','LEFT_ARM_QUAT_C','LEFT_ARM_QUAT_D']].values
                
                self.leftArmMesh = self.generateCylinder(
                    center_x = self.leftArmPos[0],
                    center_y = self.leftArmPos[1],
                    center_z = self.leftArmPos[2],
                    radius  =  self.leftArmRadius,
                    height_z  = self.leftArmHeight,
                    resolution=self.resolution) 
                # LEFT ARM ROTATION
                target_direction = np.array(self.leftHandPos - self.leftArmPos).astype(float)
                origin_direction = np.array([0.0,0.0,1.0]).astype(float)
                self.leftArmMesh = self.rotateInDirection(self.leftArmMesh, target_direction, origin_direction)
                self.leftArmOffset = self.leftArmPos - self.leftArmMesh.mean(axis=1)

                self.leftArmMesh[0, :] += self.leftArmOffset[0]
                self.leftArmMesh[1, :] += self.leftArmOffset[1]
                self.leftArmMesh[2, :] += self.leftArmOffset[2]

                
            if not np.isnan(self.data['RIGHT_HAND_POSITION_X']):
                # RIGHT HAND MESH
                self.rightHandPos = self.data[['RIGHT_HAND_POSITION_X','RIGHT_HAND_POSITION_Y', 'RIGHT_HAND_POSITION_Z']].values
                self.rightHandRadius = self.data['RIGHT_HAND_RADIUS']
                self.rightHandMesh = self.generateSphere(
                    center_x = self.rightHandPos[0],
                    center_y = self.rightHandPos[1],
                    center_z = self.rightHandPos[2],
                    radius  = self.rightHandRadius * 2,
                    resolution=self.resolution) 
                # RIGHT ARM MESH
                self.rightArmPos = self.data[['RIGHT_ARM_POSITION_X','RIGHT_ARM_POSITION_Y', 'RIGHT_ARM_POSITION_Z']].values
                self.rightArmDir = self.rightHandPos - self.rightArmPos
                rightMagnitude = np.sqrt(self.rightArmDir.dot( self.rightArmDir))
                self.rightArmPos -= self.rightArmDir * ((self.rightHandRadius*2.0)/rightMagnitude)
                self.rightArmRadius = self.data['RIGHT_ARM_RADIUS']
                self.rightArmHeight = self.data['RIGHT_ARM_HEIGHT']
                self.rightArmRot = self.data[['RIGHT_ARM_QUAT_A','RIGHT_ARM_QUAT_B','RIGHT_ARM_QUAT_C','RIGHT_ARM_QUAT_D']].values

                self.rightArmMesh = self.generateCylinder(
                    center_x = self.rightArmPos[0],
                    center_y = self.rightArmPos[1],
                    center_z = self.rightArmPos[2],
                    radius  =  self.rightArmRadius,
                    height_z  = self.rightArmHeight,
                    resolution=self.resolution) 
                # # RIGHT ARM ROTATION
                target_direction = np.array(self.rightHandPos - self.rightArmPos).astype(float)
                origin_direction = np.array([0.0,0.0,1.0]).astype(float)
                self.rightArmMesh = self.rotateInDirection(self.rightArmMesh, target_direction, origin_direction)
                self.rightArmOffset = self.rightArmPos - self.rightArmMesh.mean(axis=1)

                self.rightArmMesh[0, :] += self.rightArmOffset[0]
                self.rightArmMesh[1, :] += self.rightArmOffset[1]
                self.rightArmMesh[2, :] += self.rightArmOffset[2]
    def initialize(self):
            self.update()
            if type(self.leftHandMesh) is np.ndarray:
                #self.leftHandPlot = self.ax.scatter(self.leftHandMesh[0],self.leftHandMesh[1], self.leftHandMesh[2], marker='o', color='blue')
                self.leftHandSurf = self.ax.plot_surface(
                    self.leftHandMesh[0].reshape(self.resolution, self.resolution),
                    self.leftHandMesh[1].reshape(self.resolution, self.resolution),
                    self.leftHandMesh[2].reshape(self.resolution, self.resolution),
                    rstride=1, 
                    cstride=1, 
                    color = 'b',
                    linewidth=0, 
                    antialiased=True)
            if type(self.leftArmMesh) is np.ndarray: 
                #self.leftArmPlot = self.ax.scatter(self.leftArmMesh[0],self.leftArmMesh[1], self.leftArmMesh[2], marker='o', color='blue')
                self.leftArmSurf = self.ax.plot_surface(
                    self.leftArmMesh[0].reshape(self.resolution, self.resolution),
                    self.leftArmMesh[1].reshape(self.resolution, self.resolution),
                    self.leftArmMesh[2].reshape(self.resolution, self.resolution),
                    rstride=1, 
                    cstride=1, 
                    color = 'b',
                    linewidth=0, 
                    antialiased=True)
            if type(self.rightHandMesh) is np.ndarray:
                #self.rightHandPlot = self.ax.scatter(self.rightHandMesh[0],self.rightHandMesh[1], self.rightHandMesh[2], marker='o', color='blue')
                self.rightHandSurf = self.ax.plot_surface(
                    self.rightHandMesh[0].reshape(self.resolution, self.resolution),
                    self.rightHandMesh[1].reshape(self.resolution, self.resolution),
                    self.rightHandMesh[2].reshape(self.resolution, self.resolution),
                    rstride=1, 
                    cstride=1, 
                    color = 'b',
                    linewidth=0, 
                    antialiased=True); 
            if type(self.rightArmMesh) is np.ndarray: 
                #self.rightArmPlot = self.ax.scatter(self.rightArmMesh[0],self.rightArmMesh[1], self.rightArmMesh[2], marker='o', color='blue')
                self.rightArmSurf = self.ax.plot_surface(
                    self.rightArmMesh[0].reshape(self.resolution, self.resolution),
                    self.rightArmMesh[1].reshape(self.resolution, self.resolution),
                    self.rightArmMesh[2].reshape(self.resolution, self.resolution),
                    rstride=1, 
                    cstride=1, 
                    color = 'b',
                    linewidth=0, 
                    antialiased=True); 
    def animate(self, i):
            self.update()
            if type(self.leftHandMesh) is np.ndarray:
                self.leftHandSurf.remove()
                #self.leftHandPlot = self.ax.scatter(self.leftHandMesh[0],self.leftHandMesh[1], self.leftHandMesh[2], marker='o', color='blue')
                self.leftHandSurf = self.ax.plot_surface(
                    self.leftHandMesh[0].reshape(self.resolution, self.resolution),
                    self.leftHandMesh[1].reshape(self.resolution, self.resolution),
                    self.leftHandMesh[2].reshape(self.resolution, self.resolution),
                    rstride=1, 
                    cstride=1, 
                    color = 'b',
                    linewidth=0, 
                    antialiased=True)
            if type(self.leftArmMesh) is np.ndarray: 
                self.leftArmSurf.remove()
                #self.leftArmPlot = self.ax.scatter(self.leftArmMesh[0],self.leftArmMesh[1], self.leftArmMesh[2], marker='o', color='blue')
                self.leftArmSurf = self.ax.plot_surface(
                    self.leftArmMesh[0].reshape(self.resolution, self.resolution),
                    self.leftArmMesh[1].reshape(self.resolution, self.resolution),
                    self.leftArmMesh[2].reshape(self.resolution, self.resolution),
                    rstride=1, 
                    cstride=1, 
                    color = 'b',
                    linewidth=0, 
                    antialiased=True)
            if type(self.rightHandMesh) is np.ndarray:
                self.rightHandSurf.remove()
                #self.rightHandPlot = self.ax.scatter(self.rightHandMesh[0],self.rightHandMesh[1], self.rightHandMesh[2], marker='o', color='blue')
                self.rightHandSurf = self.ax.plot_surface(
                    self.rightHandMesh[0].reshape(self.resolution, self.resolution),
                    self.rightHandMesh[1].reshape(self.resolution, self.resolution),
                    self.rightHandMesh[2].reshape(self.resolution, self.resolution),
                    rstride=1, 
                    cstride=1, 
                    color = 'b',
                    linewidth=0, 
                    antialiased=True); 
            if type(self.rightArmMesh) is np.ndarray: 
                self.rightArmSurf.remove()
                #self.rightArmPlot = self.ax.scatter(self.rightArmMesh[0],self.rightArmMesh[1], self.rightArmMesh[2], marker='o', color='blue')
                self.rightArmSurf = self.ax.plot_surface(
                    self.rightArmMesh[0].reshape(self.resolution, self.resolution),
                    self.rightArmMesh[1].reshape(self.resolution, self.resolution),
                    self.rightArmMesh[2].reshape(self.resolution, self.resolution),
                    rstride=1, 
                    cstride=1, 
                    color = 'b',
                    linewidth=0, 
                    antialiased=True); 