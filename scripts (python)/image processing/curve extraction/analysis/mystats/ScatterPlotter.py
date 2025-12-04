import numpy as np
import pandas as pd
from scipy import stats
import time
import matplotlib.pyplot as pyplot
from colour import Color
import seaborn as sns

class ScatterPlotter:
     def __init__(self, filename, title, sheetDf, slope, intercept):
        df = sheetDf
        fig = pyplot.figure(1, figsize=(len(df.columns.values)+1, 2))
        ax = fig.add_subplot(1,1,1) #, aspect=1
        ax.spines['right'].set_visible(False)
        ax.spines['top'].set_visible(False)
        pyplot.title(r"$\bf{" + str(title) + "}$", size=14, y=1.15)
        colorRange = sns.color_palette("RdYlGn", 3)
        ax.scatter(x=df.iloc[:,0], y=df.iloc[:,1], color=colorRange[1], marker='o', s=5, edgecolors='black', linewidths=0.75)
        ax.plot(df.iloc[:,0], slope*df.iloc[:,0]+intercept, color='black', linewidth=0.75)
        pyplot.ylabel('MEASURE', size=9)
        pyplot.xlabel('MEASURE', size=9)
        ax.tick_params(axis='both', which='major', labelsize=9)
        fig.savefig(filename, bbox_inches='tight', dpi = 300)
        pyplot.close("all")
