import numpy as np
import pandas as pd
from scipy import stats
import time
import matplotlib.pyplot as pyplot
from colour import Color
import seaborn as sns
from matplotlib.font_manager import FontProperties
import pingouin as pg

import matplotlib.ticker as mticker

class StackedBarPlotter:
     def __init__(self, filename, title, dataDf, histDf, statDf):
        df = histDf/histDf.sum() * 100
        height=0.75
        fig, axs = pyplot.subplots(3, 1, sharey=False, sharex=False, figsize=(4, len(df.columns.values)+1), gridspec_kw={'height_ratios': [0.75, 0.75, 1]})
        fig.suptitle(r"$\bf{" + str(title) + "}$", size=11)
        pyplot.subplots_adjust(hspace = 1)
        #fig = pyplot.figure(1, figsize=(len(df.columns.values)+1, 1))
        #axs[0] = fig.add_subplot(2,1,1) #, aspect=1
        axs[0].spines['right'].set_visible(False)
        axs[0].spines['top'].set_visible(False)
        
        columnIndexes = np.arange(len(df.columns.values))    # the x locations for the groups
        rowIndexes = np.arange(len(df.index.values))
        colorRange = sns.color_palette("RdYlGn", len(df.index.values))

        # getting middle position
        xMidPos = 0
        data = {}
        for index in df.index.values:
          data[index] = {'x':[], 'y':[],  'label':[], 'count':[], 'width':[], 'height':[], 'left':[], 'color':[], 'edgecolor':[]}
        
        for i in columnIndexes:
          columnName = df.columns.values[i]
          dfColumn = df.loc[:,columnName]
          dfRawColumn = histDf.loc[:,columnName]
          nbIndexes = len(dfColumn.index.values)
          if nbIndexes % 2: # unpaired
            midIndex = int(nbIndexes/2)
            leftMidPos = xMidPos - dfColumn[midIndex]/2
            data[midIndex]['label'].append(columnName)
            data[midIndex]['width'].append(dfColumn[midIndex])
            data[midIndex]['count'].append(dfRawColumn[midIndex])
            data[midIndex]['left'].append(leftMidPos)
            data[midIndex]['color'].append(colorRange[midIndex])
            data[midIndex]['y'].append(i)
            data[midIndex]['x'].append(leftMidPos + dfColumn[midIndex]/2)

            for j in range(midIndex-1, -1, -1):
              leftLeftPos = leftMidPos - dfColumn[j:midIndex].sum()
              data[j]['label'].append(columnName)
              data[j]['width'].append(dfColumn[j])
              data[j]['count'].append(dfRawColumn[j])
              data[j]['left'].append(leftLeftPos)
              data[j]['color'].append(colorRange[j])
              data[j]['y'].append(i)
              data[j]['x'].append(leftLeftPos + dfColumn[j]/2)

            for j in range(midIndex+1, len(dfColumn.index.values), +1):
              leftRightPos = leftMidPos + dfColumn[midIndex:j].sum()
              data[j]['label'].append(columnName)
              data[j]['width'].append(dfColumn[j])
              data[j]['count'].append(dfRawColumn[j])
              data[j]['left'].append(leftRightPos)
              data[j]['color'].append(colorRange[j])
              data[j]['y'].append(i)
              data[j]['x'].append(leftRightPos + dfColumn[j]/2)


          else: # paired
            midIndex = int(nbIndexes/2)
            for j in range(midIndex-1, -1, -1):
              leftLeftPos = xMidPos - dfColumn[j:midIndex].sum()
              data[j]['label'].append(columnName)
              data[j]['width'].append(dfColumn[j])
              data[j]['count'].append(dfRawColumn[j])
              data[j]['left'].append(leftLeftPos)
              data[j]['color'].append(colorRange[j])
              data[j]['y'].append(i)
              data[j]['x'].append(leftLeftPos + dfColumn[j]/2)

            for j in range(midIndex, len(dfColumn.index.values), +1):
              leftRightPos = xMidPos + dfColumn[midIndex:j].sum()
              data[j]['label'].append(columnName)
              data[j]['width'].append(dfColumn[j])
              data[j]['count'].append(dfRawColumn[j])
              data[j]['left'].append(leftRightPos)
              data[j]['color'].append(colorRange[j])
              data[j]['y'].append(i)
              data[j]['x'].append(leftRightPos + dfColumn[j]/2)
        for key, value in data.items():
          axs[0].barh(y=value['label'], width=value['width'], height=height, left=value['left'], color=value['color'])

          for x, y, v, c in zip(value['x'], value['y'], value['count'], value['color']):
            if int(v):
              axs[0].text(x, y, int(v), ha='center', va='center',  size=6, color='black')
        
        # plots = []
        # prevColumnValues = np.zeros(len(columnIndexes))
        # for i in rowIndexes:
        #     rowName = df.index.values[i]
        #     columnValues = df.loc[rowName,:]
        #     fillColorValues = [colorRange[i].rgb for j in range(len(columnIndexes))]
        #     edgeColorValues = [Color('black').rgb for j in range(len(columnIndexes))]
        #     textColorValues = [textColorRange[i].rgb for j in range(len(columnIndexes))]
        #     plots.append(pyplot.barh(columnIndexes, columnValues.values, width, left=prevColumnValues, color=fillColorValues, edgecolor=edgeColorValues))

        #     for x, py, cy, t, c in zip(columnIndexes, prevColumnValues, columnValues.values, columnValues.values, textColorValues):
        #         if t:
        #             pyplot.text(x, py + cy/2, "{:.1f}%".format(t/maxValue*100), ha='center', va='center',  size=10, color=c)
        #     prevColumnValues += columnValues.values

        axs[0].legend( labels=[key for key in data], loc='lower center', frameon=False, bbox_to_anchor=(0.5, 1),fancybox=False, shadow=False, fontsize='xx-small', ncol=len(data))
        axs[0].set_xlabel('Frequency', size=9)
        #axs[0].tick_params(axis='both', which='major', labelsize=9
        #pyplot.yticks(columnIndexes, df.columns.values)
        axs[0].set_yticks(columnIndexes)
        axs[0].set_yticklabels(['%s' %column_name for column_name in df.columns.values])
        axs[0].set_xlim([-100, 100])
        vals = axs[0].get_xticks()
        
        ticks_loc = axs[0].get_xticks().tolist()
        axs[0].xaxis.set_major_locator(mticker.FixedLocator(ticks_loc))

        #axs[0].set_xticklabels([label_format.format(x) for x in ticks_loc])
        axs[0].set_xticklabels([ str(int(abs(x))) + '%' for x in vals])
        axs[0].tick_params(axis='y', labelsize=9)
        axs[0].tick_params(axis='x', labelsize=7)

        # Figure 2
        for i in columnIndexes:
          columnName = df.columns.values[i]
          # Compute MEAN and CI
          column_mean = dataDf[columnName].mean()
          column_ci = pg.compute_bootci(dataDf[columnName],func='mean')
          print('%s -> %f[%f %f](95 CI)' %(columnName, column_mean, column_ci[0], column_ci[1]))
          axs[1].plot(column_mean, i, 'o', markersize=6, markerfacecolor='white', markeredgecolor='black', markeredgewidth=1.5) 
          axs[1].hlines(columnName, column_ci[0], column_ci[1], colors='black', linestyles='solid', linewidth=1.5)
          axs[1].barh(y=columnName, width=0.1, height=height, left=0, color='white')
        #print(histDf.index.asttypvalues.tolist())
        #print(dataDf.index.values)
        axs[1].set_xlabel('Score', size=9)
        axs[1].set_yticks(columnIndexes)
        axs[1].set_yticklabels(['%s' %column_name for column_name in df.columns.values])
        axs[1].set_xticks(histDf.index.astype(float).values)

        axs[1].spines['right'].set_visible(False)
        axs[1].spines['top'].set_visible(False)

        axs[1].tick_params(axis='y', labelsize=9)
        axs[1].tick_params(axis='x', labelsize=7)

        # Figure 3
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

        # save image
        fig.savefig(filename, bbox_inches='tight', dpi = 300)
        pyplot.close("all")
