import numpy as np
import pandas as pd
from scipy import stats
import time
import matplotlib.pyplot as pyplot
from matplotlib import animation
from colour import Color
import seaborn as sns


class HeatMapPlotter:
    def __init__(self, sheetDf, nbRows, nbColumns):
        self.df = sheetDf
        self.dimension = (nbRows, nbColumns)
        self.data = np.zeros(self.dimension)
        self.vmax = self.df.max().max()
        print(self.vmax)

        pyplot.rcParams["figure.figsize"] = [7.50, 3.50]
        pyplot.rcParams["figure.autolayout"] = True

        fig = pyplot.figure()
        sns.heatmap(self.data, vmax=self.vmax, square=True)
        
        anim = animation.FuncAnimation(fig, self.animate, init_func=self.initialize, frames=self.df.shape[0], repeat=False)
        pyplot.show()
    
    def initialize(self):
        sns.heatmap(np.zeros(self.dimension), vmax= self.vmax, cbar=False, square=True)

    def animate(self, i):
        
        self.data = self.df.head(1).values.reshape(self.dimension)
        print(self.data)
        self.df = self.df.iloc[1:]
        sns.heatmap(self.data, vmax= self.vmax, cbar=False, square=True)
        a.ax_joint.plot([15],[3],'o',ms=60,mec='r',mfc='none')
