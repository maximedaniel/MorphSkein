import numpy as np
import pandas as pd
from scipy import stats
import time
import matplotlib.pyplot as pyplot
from colour import Color
import seaborn as sns

class SignalPlotter:
     def __init__(self, filename, title, df_eda_signals, df_bvp_signals, df_sc_signals, plotting, saving):
        self.filename = filename
        self.title = title
        self.df_eda_signals =  df_eda_signals
        self.df_bvp_signals =  df_bvp_signals
        self.df_sc_signals =  df_sc_signals
        self.plotting = plotting
        self.saving = saving
        #fig = pyplot.figure(1, figsize=(len(df.columns.values)+1, 1))

        #create subplot figure with having two side by side plots
        
        fig, axes = pyplot.subplots(nrows=8,ncols=1,figsize=(6,6))
        # plot first pandas frame in subplot style
        #eda_plot = nk.events_plot(events= eda_events, signal = eda_sub_epoch, show=False)  # Plot scaled signals"
        self.df_eda_signals.plot(ax = axes[:3], subplots=True, legend=True)  # Plot scaled signals"
        for i in range(self.df_eda_signals.shape[1]):
            x0 = 0
            x1 = 0
            y0 = self.df_eda_signals.iloc[:, i].min()
            y1 = self.df_eda_signals.iloc[:, i].max()
            axes[i].plot([x0, x1], [y0, y1], color='k', linestyle='-', linewidth=2)
        
        # plot second pandas frame in subplot style
        #bvp_plot = nk.events_plot(events= bvp_events, signal = bvp_sub_epoch, show=False)  # Plot scaled signals"
        self.df_bvp_signals.plot(ax = axes[3:6], subplots=True, legend=True)  # Plot scaled signals"
        for i in range(self.df_bvp_signals.shape[1]):
            x0 = 0
            x1 = 0
            y0 = self.df_bvp_signals.iloc[:, i].min()
            y1 = self.df_bvp_signals.iloc[:, i].max()
            axes[i + 3].plot([x0, x1], [y0, y1], color='k', linestyle='-', linewidth=2)
        # plot third pandas frame in subplot style
        self.df_sc_signals.plot(ax = axes[6:], subplots=True, legend=True)  # Plot scaled signals"
        for i in range(self.df_sc_signals.shape[1]):
            x0 = 0
            x1 = 0
            y0 = self.df_sc_signals.iloc[:, i].min()
            y1 = self.df_sc_signals.iloc[:, i].max()
            axes[i + 6].plot([x0, x1], [y0, y1], color='k', linestyle='-', linewidth=2)
        if self.plotting:
            pyplot.show()
        if self.saving:
            fig.savefig(filename, bbox_inches='tight', dpi = 300)
        pyplot.close("all")
