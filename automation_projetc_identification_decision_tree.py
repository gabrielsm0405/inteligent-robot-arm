# -*- coding: utf-8 -*-
"""automation_projetc_identification.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1KmhEKx06yrfkPiliMNs4bOOz1Ehw5jF5
"""

# Installing TPOT
!pip install sklearn-genetic-opt

# Importing necessary libraries
import numpy as np
import pandas as pd
from sklearn.tree import DecisionTreeRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import get_scorer_names, mean_squared_error
from sklearn_genetic import GASearchCV
from sklearn.model_selection import KFold
from sklearn_genetic.space import Continuous, Categorical, Integer
import matplotlib.pyplot as plt
from sklearn_genetic.plots import plot_fitness_evolution, plot_search_space
from joblib import dump, load
import warnings
warnings.filterwarnings('ignore')

# Reading the dataset
data=pd.read_csv('funcModel.csv')
data_utils=data.dropna(axis=0)
data_utils[:5]

X=data_utils.drop(['gx', 'gy', 'gz'],axis=1)
y = data_utils.drop(['a1', 'a2', 'a3', 'a4'],axis=1)

# Splitting the data for train and test in standard ratio 70:30 respectively.

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.30, random_state=42, shuffle=True)

clf = DecisionTreeRegressor()

# Defining the parameters for the optimization of the hyperparameter

param_grid = {
              'criterion': Categorical (choices = ['squared_error', 'friedman_mse']),
              'splitter': Categorical (choices = ['best', 'random']),
              'max_depth': Integer(2, 30), 
              'min_samples_split': Integer(2, 14),
              'min_samples_leaf': Integer(2, 8),
              'min_weight_fraction_leaf': Continuous(0.01, 0.5, distribution='log-uniform'),
              'max_features': Categorical (choices = ['auto', 'sqrt', 'log2'])
              }

cv = KFold(n_splits=3, shuffle=True)

evolved_estimator = GASearchCV(
                               estimator=clf, # Estimator object
                               cv=cv, # Cross-validation generator
                               scoring='neg_mean_squared_error', # Strategy to evaluate the performance of the cross-validated model on the test set
                               population_size=10, # Size of the initial population
                               generations=35, # Number of generations
                               tournament_size=3, # Number of individuals to perform tournament selection
                               elitism=True, # If True takes the tournament_size best solution to the next generation
                               crossover_probability=0.8, # Probability of crossover
                               mutation_probability=0.1, # Probability of child mutation
                               param_grid=param_grid, # Grid with the parameters to tune
                               criteria='max', # max if a higher scoring metric is better, min otherwise.
                               algorithm='eaMuPlusLambda', # Evolutionary algorithm to use. {‘eaMuPlusLambda’, ‘eaMuCommaLambda’, ‘eaSimple’}
                               n_jobs=-1, # Number of jobs to run in parallel. -1 means using all processors
                               verbose=True, # If True, shows the metrics on the optimization routine
                               keep_top_k=4, # Number of best solutions to keep in the hof object
                               error_score="raise")

evolved_estimator.fit(X_train,y_train)

y_predicy_ga = evolved_estimator.predict(X_test)
mean_squared_error(y_test,y_predicy_ga)

evolved_estimator.best_params_

plot_fitness_evolution(evolved_estimator)
plt.show()

print("Parameters and cv scores in each iteration:")
print(evolved_estimator.logbook)

plot_search_space(evolved_estimator)
plt.show()

print("Best K solutions:")
print(evolved_estimator.hof)

dump(clf, 'identification.joblib')