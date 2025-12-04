import numpy as np
import pandas as pd
from scipy import stats
import time
import matplotlib.pyplot as pyplot
from matplotlib.font_manager import FontProperties
from colour import Color
import seaborn as sns
import pingouin as pg

MAX_STAT_ROWS = 10
class BoxPlotter:
     def __init__(self, filename, title, sheetDf, statDf=None):
        # Figure 1
        df = sheetDf
        height=0.75
        fig, axs = pyplot.subplots(3, 1, sharey=False, sharex=False, figsize=(len(df.columns.values)+1, 5))
        pyplot.subplots_adjust(hspace = 0.5)
        
        axs[0].spines['right'].set_visible(False)
        axs[0].spines['top'].set_visible(False)
        axs[0].set_title(r"$\bf{" + str(title) + "}$", size=11, y=1.15)

        #pyplot.title(r"$\bf{" + str(title) + "}$", size=14, y=1.15)
        colorRange = sns.color_palette("RdYlGn", 7)
        columnIndexes = np.arange(len(df.columns.values))    # the x locations for the groups
        data = [ df.loc[~np.isnan(df[factor]), factor] for factor in df.columns.values]
        axs[0].boxplot(
          data,
          notch=False,
          vert=False,
          positions=columnIndexes,
          widths=[ height for columnValue in df.columns.values],
          #patch_artist=True,
          #boxprops=dict(facecolor=colorRange[3], color=colorRange[-1]),
          #capprops=dict(color=colorRange[-1]),
          #whiskerprops=dict(color=colorRange[-1]),
          #flierprops=dict(color=colorRange[0], markeredgecolor=colorRange[0]),
          flierprops=dict(color=colorRange[0], markeredgecolor=colorRange[0], markersize=1),
          #medianprops=dict(color=colorRange[0])
        )
        axs[0].set_xlabel('MEASURE', size=9)
        axs[0].tick_params(axis='both', which='major', labelsize=9)
        axs[0].set_yticks(columnIndexes)
        axs[0].set_yticklabels(['%s' %column_name for column_name in df.columns.values])
        #ax.set_xlim([-100, 100])
        #vals = ax.get_xticks()
        #ax.set_xticklabels([ str(int(abs(x))) + '%' for x in vals])
        
        # Figure 2
        for i in columnIndexes:
          columnName = df.columns.values[i]
          # Compute MEAN and CI
          column_mean = sheetDf[columnName].mean()
          column_ci = pg.compute_bootci(sheetDf.loc[~np.isnan(sheetDf[columnName]), columnName],func='mean')
          # print('%s -> %f[%f %f](95 CI)' %(columnName, column_mean, column_ci[0], column_ci[1]))
          axs[1].plot(column_mean, i, 'o', markersize=6, markerfacecolor='white', markeredgecolor='black', markeredgewidth=1.5) 
          axs[1].hlines(columnName, column_ci[0], column_ci[1], colors='black', linestyles='solid', linewidth=1.5)
          axs[1].barh(y=columnName, width=0.1, height=height, left=0, color='white')
        #print(histDf.index.asttypvalues.tolist())
        #print(dataDf.index.values)
        axs[1].set_xlabel('Score', size=9)
        axs[1].set_yticks(columnIndexes)
        axs[1].set_yticklabels(['%s' %column_name for column_name in df.columns.values])
        # axs[1].set_xticks(histDf.index.astype(float).values)

        axs[1].spines['right'].set_visible(False)
        axs[1].spines['top'].set_visible(False)

        axs[1].tick_params(axis='y', labelsize=9)
        axs[1].tick_params(axis='x', labelsize=7)

        # Figure 3
        if statDf.shape[0] <= MAX_STAT_ROWS:
          statDf = statDf.round(4)
          table = axs[2].table(cellText=statDf.values, colLabels=statDf.columns.values, loc='center')
          table.auto_set_font_size(False)
          table.scale(1.2, 1)
          axs[2].axis('off')
          for j in range(len(statDf.columns.values)):
            table[(0, j)].set_edgecolor("none")
            table[(0, j)].set_text_props(color='white', size=5, fontproperties=FontProperties(weight='bold'))
            table[(0, j)].set_facecolor("#000000")
          
          for i in range(len(statDf.index.values)):
            for j in range(len(statDf.columns.values)):
              values = statDf.iloc[i].values
              table[(i+1, j)].set_text_props(color='black', size=5)
              table[(i+1, j)].set_edgecolor("none")
              if isinstance(values[3], float) and values[3] < .05:
                #table[(i+1, j)].set_text_props(color='white')
                table[(i+1, j)].set_facecolor(colorRange[-1])
              else :
                table[(i+1, j)].set_facecolor('#F8F8F8') #colorRange[int(len(colorRange)/2)]  
        fig.savefig(filename, bbox_inches='tight', dpi = 300)
        pyplot.close("all")
