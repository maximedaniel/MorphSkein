
import numpy as np
import pandas as pd
from scipy import stats
import time
import statsmodels.api as sm
import statsmodels.stats.descriptivestats as smdesc
import math
import os
import sys
import pingouin as pg


from . import StackedBarPlotter
from . import ScatterPlotter
from .  import BoxPlotter

# if not os.path.exists(imgDir):
#     os.makedirs(imgDir)


class Statistics:
    
    @staticmethod
    def describePlus(sheetDf):
        description = sheetDf.describe()
        # compute confidence interval
        low_ci = []
        high_ci = []
        for columnName in sheetDf.columns.values:
          column_ci = pg.compute_bootci(sheetDf.loc[~np.isnan(sheetDf[columnName]), columnName],func='mean')
          low_ci.append(column_ci[0])
          high_ci.append(column_ci[1])
        description.loc['cil'] = low_ci
        description.loc['cih'] = high_ci
        return description

    @staticmethod
    def normalityAsumption(sheetDf):
        normalityDf = pd.DataFrame(columns=sheetDf.columns.values)
        statistics = []
        pvalues = []
        areNormal = []
        for columnName in sheetDf.columns.values:
            normality_stats = pg.normality(sheetDf.loc[~np.isnan(sheetDf[columnName]), columnName])
            statistics.append(normality_stats['W'].values[0])
            pvalues.append(normality_stats['pval'].values[0])
            areNormal.append(normality_stats['normal'].values[0])
        normalityDf.loc['stat'] = statistics
        normalityDf.loc['pval'] = pvalues
        normalityDf.loc['norm'] = areNormal
        return normalityDf

    @staticmethod
    def qualNominalPaired(imgDir, sheetName, sheetDf):
        print("######################################## ",sheetName," ########################################")
        meltedSheetDf = sheetDf.melt(var_name='columns', value_name='index')
        contingencySheetTable = pd.crosstab(index=meltedSheetDf['index'], columns=meltedSheetDf['columns'])
        #if len(splittedSheetName) > 1:
        #     orderedColumns = splittedSheetName[1].split('>')
        #     contingencySheetTable = contingencySheetTable.reindex(orderedColumns)
        contingencySheetTable.loc['COUNT'] = contingencySheetTable.sum().values
        contingencySheetTable = contingencySheetTable.drop(index='COUNT')
        
        if len(sheetDf.columns) > 2:
            print(contingencySheetTable)
            chi2, pvalue, dof, ex = stats.chi2_contingency(contingencySheetTable.T)
            print( sheetDf.columns.str.cat(sep=' | '), " -> CHI² (statistic: ", chi2, ", p-value: ", pvalue,")")
        for i in range(len(sheetDf.columns.values)):
            for j in range(i+1, len(sheetDf.columns.values)):
                try:
                    ctab = contingencySheetTable[[sheetDf.columns.values[i], sheetDf.columns.values[j]]]
                    print(ctab)
                    chi2, pvalue, dof, ex = stats.chi2_contingency(ctab)
                    print(sheetDf.columns.values[i],'|', sheetDf.columns.values[j],  " -> CHI² (statistic: ", chi2, ", p-value: ", pvalue,")")
                except ValueError as ChiError:
                    print(sheetDf.columns.values[i],'|', sheetDf.columns.values[j],  " -> CHI² (",ChiError,")")
                    try:
                        ctab = contingencySheetTable[[sheetDf.columns.values[i], sheetDf.columns.values[j]]]
                        oddsratio, pvalue = stats.fisher_exact(ctab)
                        print(sheetDf.columns.values[i],'|', sheetDf.columns.values[j], " -> Fisher (statistic: ", oddsratio, ", p-value: ", pvalue,")")
                    except ValueError as FisherError:
                        print(sheetDf.columns.values[i],'|', sheetDf.columns.values[j], " -> Fisher (", FisherError,")")
    
    @staticmethod
    def qualNominalUnpaired(imgDir,  sheetName, sheetDf):
        print("######################################## ",sheetName," ########################################")
        meltedSheetDf = sheetDf.melt(var_name='columns', value_name='index')
        contingencySheetTable = pd.crosstab(index=meltedSheetDf['index'], columns=meltedSheetDf['columns'])
        #if len(splittedSheetName) > 1:
        #    orderedColumns = splittedSheetName[1].split('>')
        #    contingencySheetTable = contingencySheetTable.reindex(orderedColumns)
        contingencySheetTable.loc['COUNT'] = contingencySheetTable.sum().values
        
        if len(sheetDf.columns) > 2:
            print(contingencySheetTable)
            contingencySheetTable = contingencySheetTable.drop(index='COUNT')
            chi2, pvalue, dof, ex = stats.chi2_contingency(contingencySheetTable.T)
            print( sheetDf.columns.str.cat(sep=' | '), " -> CHI² (statistic: ", chi2, ", p-value: ", pvalue,")")
        
        for i in range(len(sheetDf.columns.values)):
            for j in range(i+1, len(sheetDf.columns.values)):
                try:
                    ctab = contingencySheetTable[[sheetDf.columns.values[i], sheetDf.columns.values[j]]]
                    print(ctab)
                    chi2, pvalue, dof, ex = stats.chi2_contingency(ctab)
                    print(sheetDf.columns.values[i],'|', sheetDf.columns.values[j],  " -> CHI² (statistic: ", chi2, ", p-value: ", pvalue,")")
                except ValueError as ChiError:
                    print(sheetDf.columns.values[i],'|', sheetDf.columns.values[j],  " -> CHI² (",ChiError,")")
                    try:
                        ctab = contingencySheetTable[[sheetDf.columns.values[i], sheetDf.columns.values[j]]]
                        oddsratio, pvalue = stats.fisher_exact(ctab)
                        print(sheetDf.columns.values[i],'|', sheetDf.columns.values[j], " -> Fisher (statistic: ", oddsratio, ", p-value: ", pvalue,")")
                    except ValueError as FisherError:
                        print(sheetDf.columns.values[i],'|', sheetDf.columns.values[j], " -> Fisher (", FisherError,")")
        StackedBarPlotter.StackedBarPlotter(
         filename =  imgDir + '/' + sheetName + '.png', 
         title = sheetName,
         sheetDf = contingencySheetTable)

    @staticmethod
    def qualOrdinalPaired(imgDir,  sheetName, sheetDf, sheetScale, silent=True):
        print("######################################## ",sheetName," ########################################") if not silent else None
        print(Statistics.describePlus(sheetDf)) if not silent else None
        meltedSheetDf = sheetDf.melt(var_name='Factor', value_name='Variable')
        contingencySheetDf = pd.crosstab(index=meltedSheetDf['Variable'], columns=meltedSheetDf['Factor'])
        statDf = pd.DataFrame(columns=['COMPARISON', 'TEST', 'STATISTICS', 'P-VALUE', 'EFFECT SIZE'])
        #fill empty scale value
        for sheetStep in range(sheetScale):
            if not sheetStep in contingencySheetDf.index.values:
                contingencySheetDf.loc[sheetStep] = [0 for x in range(len(contingencySheetDf.columns.values))]
        contingencySheetDf.sort_index(inplace=True)

        # ALL MODALITY
        if len(contingencySheetDf.columns) > 2:
            sheetDf_long = sheetDf.melt(ignore_index=False).reset_index()
            friedman_stats = pg.friedman(data=sheetDf_long, dv="value", within="variable", subject="index")
            source, wvalue, ddof1, qvalue, pvalue = friedman_stats.values[0]
            statDf = statDf.append(
                {'COMPARISON': 'ALL',
                'TEST': "Friedman",
                'STATISTICS':qvalue,
                'P-VALUE':pvalue,
                'EFFECT SIZE':wvalue
                }, ignore_index=True)
            print(sheetDf.columns.str.cat(sep=' | '), " -> Friedman (statistic:", qvalue, " p-value: ", pvalue, ", effect size:", wvalue, ")") if not silent else None
                
        # BETWEEN MODALITY
        modality_names = sheetDf.columns.values
        uncorrectedStatIndex = len(statDf.index)
        for i in range(len(modality_names)):
            for j in range(i+1, len(modality_names)):
                    stats_wilcoxon = pg.wilcoxon(sheetDf.loc[:, modality_names[i]], sheetDf.loc[:, modality_names[j]],  alternative='two-sided')
                    wvalue, alternative, pvalue, RBC, CLES = stats_wilcoxon.values[0]
                    statDf = statDf.append(
                        {
                            'COMPARISON': modality_names[i] + '|' + modality_names[j],
                            'TEST': "Wilcoxon",
                            'STATISTICS':wvalue,
                            'P-VALUE':pvalue,
                            'EFFECT SIZE': RBC
                        }, ignore_index=True)
        reject, statDf.loc[uncorrectedStatIndex::,'P-VALUE'] = pg.multicomp(statDf.loc[uncorrectedStatIndex::,'P-VALUE'].values, alpha=0.05, method="bonf")
        for i in range(len(modality_names)):
            for j in range(i+1, len(modality_names)):
                    stats = statDf.loc[statDf['COMPARISON'] == sheetDf.columns.values[i] + '|' +  sheetDf.columns.values[j], :]
                    print(stats['COMPARISON'].values[0], " -> Wilcoxon (statistic:", stats['STATISTICS'].values[0], " p-value: ", stats['P-VALUE'].values[0], ", effectsize:", stats['EFFECT SIZE'].values[0], ")") if not silent else None
                
        StackedBarPlotter.StackedBarPlotter(
         filename =  imgDir + '/' + sheetName + '.png', 
         title = sheetName,
         dataDf = sheetDf,
         histDf = contingencySheetDf,
         statDf = statDf)

    @staticmethod
    def qualOrdinalUnpaired(imgDir, sheetName, sheetDf, sheetScale, silent=False):
        print("######################################## ",sheetName," ########################################") if not silent else None
        print(Statistics.describePlus(sheetDf)) if not silent else None
        meltedSheetDf = sheetDf.melt(var_name='Factor', value_name='Variable')
        contingencySheetDf = pd.crosstab(index=meltedSheetDf['Variable'], columns=meltedSheetDf['Factor'])
        statDf = pd.DataFrame(columns=['COMPARISON', 'TEST', 'STATISTICS', 'P-VALUE', 'EFFECT SIZE'])
        #fill empty scale value
        for sheetStep in range(sheetScale):
            if not sheetStep in contingencySheetDf.index.values:
                contingencySheetDf.loc[sheetStep] = [0 for x in range(len(contingencySheetDf.columns.values))]
        contingencySheetDf.sort_index(inplace=True)
        # ALL MODALITY
        if len(contingencySheetDf.columns) > 2:
            sheetDf_long = sheetDf.melt(ignore_index=False).reset_index()
            kruskal_stats = pg.kruskal(data=sheetDf_long, dv="value", between="variable")
            source, ddof1, hvalue, pvalue = kruskal_stats.values[0]
            statDf = statDf.append(
                {'COMPARISON': 'ALL',
                'TEST': "Kruskal-Wallis",
                'STATISTICS':hvalue,
                'P-VALUE':pvalue,
                'EFFECT SIZE':-1
                }, ignore_index=True)

        # BETWEEN MODALITY
        modality_names = sheetDf.columns.values
        uncorrectedStatIndex = len(statDf.index)
        for i in range(len(modality_names)):
            for j in range(i+1, len(modality_names)):
                    stats_mannwhitney = pg.mwu(x=sheetDf.loc[:, modality_names[i]], y=sheetDf.loc[:, modality_names[j]], alternative='two-sided')
                    uvalue, alternative, pvalue, RBC, CLES = stats_mannwhitney.values[0]
                    statDf = statDf.append(
                        {
                            'COMPARISON': modality_names[i] + '|' + modality_names[j],
                            'TEST': "Mann-Whitney",
                            'STATISTICS':uvalue,
                            'P-VALUE':pvalue,
                            'EFFECT SIZE': RBC
                        }, ignore_index=True)
        reject, statDf.loc[uncorrectedStatIndex::,'P-VALUE'] = pg.multicomp(statDf.loc[uncorrectedStatIndex::,'P-VALUE'].values, alpha=0.05, method="bonf")

        StackedBarPlotter.StackedBarPlotter(
         filename =  imgDir + '/' + sheetName + '.png', 
         title = sheetName,
         dataDf = sheetDf,
         histDf = contingencySheetDf,
         statDf = statDf)
    
    # sheetDf -> Column: [Factor0, Factor1, Factor2, ...]
    @staticmethod
    def quantPaired(imgDir, sheetName, sheetDf, showDf=False, silent=True):
        print("######################################## ",sheetName," ########################################") if not silent else None
        print(Statistics.describePlus(sheetDf)) if not silent else None
        normalityDf = Statistics.normalityAsumption(sheetDf)
        allColumnsAreNormal = normalityDf.loc['norm'].sum()
        print(normalityDf) if not silent else None
        statDf = pd.DataFrame(columns=['COMPARISON', 'TEST', 'STATISTICS', 'P-VALUE', 'EFFECT SIZE'])
        if allColumnsAreNormal == len(normalityDf.columns.values):
            print("Normality assumed")
            if len(sheetDf.columns) > 2:
                print(sheetDf) if showDf else None
                aov = pg.rm_anova(sheetDf) # two-ways repeated-measures ANOVA
                statistic = aov['F'].values[0]
                pvalue = aov['p-GG-corr'].values[0] if 'p-GG-corr' in aov.columns.values else aov['p-unc'].values[0]
                effsize = aov['np2'].values[0]
                print( sheetDf.columns.str.cat(sep=' | '), " -> ANOVA (statistic:", statistic, " p-value: ", pvalue, ")") if not silent else None
                statDf = statDf.append(
                    {'COMPARISON': 'ALL',
                    'TEST': "ANOVA",
                    'STATISTICS':statistic,
                    'P-VALUE':pvalue,
                    'EFFECT SIZE':effsize
                    }, ignore_index=True)
            for i in range(len(sheetDf.columns.values)):
                for j in range(i+1, len(sheetDf.columns.values)):
                    try:
                        df = sheetDf[[sheetDf.columns.values[i], sheetDf.columns.values[j]]]
                        print(df) if showDf else None
                        statistic, pvalue = stats.ttest_ind(
                            *[
                                    df.loc[~np.isnan(df[factor]), factor] 
                                    for factor in df.columns.values
                            ]
                            )
                        ttest_stats = pg.ttest(df[df.columns[0]], df[df.columns[1]], paired=True)
                        statistic = ttest_stats['T'].values[0]
                        pvalue = ttest_stats['p-val'].values[0]
                        effsize = ttest_stats['cohen-d'].values[0]
                        print(sheetDf.columns.values[i],'|', sheetDf.columns.values[j],  " -> Student (statistic: ", statistic, ", p-value: ", pvalue,")")  if not silent else None
                        statDf = statDf.append(
                        {'COMPARISON': sheetDf.columns.values[i] + '|' + sheetDf.columns.values[j],
                        'TEST': "Student",
                        'STATISTICS':statistic,
                        'P-VALUE':pvalue,
                        'EFFECT SIZE':effsize
                        }, ignore_index=True)
                    except ValueError as StudentError:
                        print(sheetDf.columns.values[i],'|', sheetDf.columns.values[j],  " -> Student (",StudentError,")") if not silent else None
                        statDf = statDf.append(
                        {'COMPARISON': sheetDf.columns.values[i] + '|' + sheetDf.columns.values[j],
                        'TEST': "Student",
                        'STATISTICS':-1,
                        'P-VALUE':-1,
                        'EFFECT SIZE':-1
                        }, ignore_index=True)
        else :
            print("Normality not assumed")
             # ALL MODALITY
            if len(sheetDf.columns) > 2:
                sheetDf_long = sheetDf.melt(ignore_index=False).reset_index()
                friedman_stats = pg.friedman(data=sheetDf_long, dv="value", within="variable", subject="index")
                source, wvalue, ddof1, qvalue, pvalue = friedman_stats.values[0]
                statDf = statDf.append(
                    {'COMPARISON': 'ALL',
                    'TEST': "Friedman",
                    'STATISTICS':qvalue,
                    'P-VALUE':pvalue,
                    'EFFECT SIZE':wvalue
                    }, ignore_index=True)
                print(sheetDf.columns.str.cat(sep=' | '), " -> Friedman (statistic:", qvalue, " p-value: ", pvalue, ", effect size:", wvalue, ")") if not silent else None
                

            # BETWEEN MODALITY
            modality_names = sheetDf.columns.values
            uncorrectedStatIndex = len(statDf.index)
            for i in range(len(modality_names)):
                for j in range(i+1, len(modality_names)):
                    x = sheetDf.loc[:, modality_names[i]]
                    y =  sheetDf.loc[:, modality_names[j]]
                    stats_wilcoxon = pg.wilcoxon(x,y, alternative='two-sided')
                    wvalue, alternative, pvalue, rbc, CLES = stats_wilcoxon.values[0]
                    statDf = statDf.append(
                            {
                                'COMPARISON': modality_names[i] + '|' + modality_names[j],
                                'TEST': "Wilcoxon",
                                'STATISTICS':wvalue,
                                'P-VALUE':pvalue,
                                'EFFECT SIZE': rbc
                            }, ignore_index=True)
            reject, statDf.loc[uncorrectedStatIndex::,'P-VALUE'] = pg.multicomp(statDf.loc[uncorrectedStatIndex::,'P-VALUE'].values, alpha=0.05, method="bonf")
            for i in range(len(modality_names)):
                for j in range(i+1, len(modality_names)):
                    stats = statDf.loc[statDf['COMPARISON'] == sheetDf.columns.values[i] + '|' +  sheetDf.columns.values[j], :]
                    print(stats['COMPARISON'].values[0], " -> Wilcoxon (statistic:", stats['STATISTICS'].values[0], " p-value: ", stats['P-VALUE'].values[0], ", effectsize:", stats['EFFECT SIZE'].values[0], ")") if not silent else None
                
        BoxPlotter.BoxPlotter(
        filename =  imgDir + '/' + sheetName + '.png', 
        title = sheetName,
        sheetDf = sheetDf,
        statDf = statDf)

        

    @staticmethod
    def quantUnpaired(imgDir, sheetName, sheetDf, showDf=False, silent=True):
        print("######################################## ",sheetName," ########################################") if not silent else None
        print(Statistics.describePlus(sheetDf)) if not silent else None
        normalityDf = Statistics.normalityAsumption(sheetDf)
        print(normalityDf) if not silent else None
        statDf = pd.DataFrame(columns=['COMPARISON', 'TEST', 'STATISTICS', 'P-VALUE', 'EFFECT SIZE'])
        if len(sheetDf.columns) > 2:
            print(sheetDf) if showDf else None
            aov = pg.rm_anova(sheetDf)
            statistic = aov['F'].values[0]
            pvalue = aov['p-GG-corr'].values[0] if 'p-GG-corr' in aov.columns.values else aov['p-unc'].values[0]
            effsize = aov['np2'].values[0]
            print( sheetDf.columns.str.cat(sep=' | '), " -> ANOVA (statistic:", statistic, " p-value: ", pvalue, ")") if not silent else None
            statDf = statDf.append(
                {'COMPARISON': 'ALL',
                'TEST': "ANOVA",
                'STATISTICS':statistic,
                'P-VALUE':pvalue,
                'EFFECT SIZE':effsize
                }, ignore_index=True)
        for i in range(len(sheetDf.columns.values)):
            for j in range(i+1, len(sheetDf.columns.values)):
                try:
                    df = sheetDf[[sheetDf.columns.values[i], sheetDf.columns.values[j]]]
                    print(df) if showDf else None
                    statistic, pvalue = stats.ttest_ind(
                        *[
                                df.loc[~np.isnan(df[factor]), factor] 
                                for factor in df.columns.values
                        ]
                        )
                    ttest_stats = pg.ttest(df[df.columns[0]], df[df.columns[1]], paired=False)
                    statistic = ttest_stats['T'].values[0]
                    pvalue = ttest_stats['p-val'].values[0]
                    effsize = ttest_stats['cohen-d'].values[0]
                    print(sheetDf.columns.values[i],'|', sheetDf.columns.values[j],  " -> Student (statistic: ", statistic, ", p-value: ", pvalue,")")  if not silent else None
                    statDf = statDf.append(
                    {'COMPARISON': sheetDf.columns.values[i] + '|' + sheetDf.columns.values[j],
                    'TEST': "Student",
                    'STATISTICS':statistic,
                    'P-VALUE':pvalue,
                    'EFFECT SIZE':effsize
                    }, ignore_index=True)
                except ValueError as StudentError:
                    print(sheetDf.columns.values[i],'|', sheetDf.columns.values[j],  " -> Student (",StudentError,")") if not silent else None
                    statDf = statDf.append(
                    {'COMPARISON': sheetDf.columns.values[i] + '|' + sheetDf.columns.values[j],
                    'TEST': "Student",
                    'STATISTICS':-1,
                    'P-VALUE':-1,
                    'EFFECT SIZE':-1
                    }, ignore_index=True)
        BoxPlotter.BoxPlotter(
         filename =  imgDir + '/' + sheetName + '.png', 
         title = sheetName,
         sheetDf = sheetDf,
         statDf = statDf)
         
    @staticmethod
    def biQuantUnpaired(imgDir, sheetName, sheetDf, describeDf=False, silent = True):
        print("######################################## ",sheetName," ########################################")  if not silent else None
        print(Statistics.describePlus(sheetDf)) if describeDf else None 
        try:
            print(sheetDf)  if not silent else None
            coefficient, pvalue = stats.pearsonr(sheetDf.iloc[:,0], sheetDf.iloc[:,1])
            print(sheetDf.columns.values[0],'|', sheetDf.columns.values[1], " -> Pearson Correlation (coefficient: ", coefficient,", pvalue: ", pvalue, ")")  if not silent else None
            try:
                slope, intercept, r_value, p_value, std_err = stats.linregress(sheetDf.iloc[:,0], sheetDf.iloc[:,1])
                print(sheetDf.columns.values[0],'|', sheetDf.columns.values[1], " -> Linear Regression (slope: ",slope, ", intercept: ", intercept, ", r_value: ", r_value, ", p_value: ", p_value, ", std_err: ", std_err, ")")  if not silent else None
            except ValueError as LinearRegressError:
                print(sheetDf.columns.values[0],'|', sheetDf.columns.values[1], " -> Linear Regression (",LinearRegressError,")")  if not silent else None
        except ValueError as PearsonError:
            print(sheetDf.columns.values[0],'|', sheetDf.columns.values[1], " -> Pearson Correlation (",PearsonError,")")  if not silent else None
       
        ScatterPlotter.ScatterPlotter(
         filename =  imgDir + '/' + sheetName + '.png', 
         title = sheetName,
         sheetDf = sheetDf,
         slope= slope,
         intercept= intercept)

         
# # Parse .xlsx file
# dictDf = pd.read_excel(dataFile, sheet_name=None)
# # Iterating key, value over Excel dictionary
# for sheetName, sheetDf in dictDf.items():
#     splittedSheetName = sheetName.split('|')
#     statType = splittedSheetName[1]
#     switcher = {
#         'QNP': Statistic.qualNominalPaired,
#         'QNU': Statistic.qualNominalUnpaired,
#         'QOP': Statistic.qualOrdinalPaired,
#         'QOU': Statistic.qualOrdinalUnpaired,
#         'QP': Statistic.quantPaired,
#         'QU': Statistic.quantUnpaired,
#         '2QU': Statistic.biQuantUnpaired,
#     }
#     statFunc = switcher.get(statType, lambda: "Bad statType")
#     statFunc(splittedSheetName[0], sheetDf)


