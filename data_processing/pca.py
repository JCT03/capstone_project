import pandas as pd
from sklearn.decomposition import PCA
import numpy as np
import os

path=input("Enter path to folder with split data\n")
training_df = pd.read_csv(path + "/training.csv", header=None)
testing_df = pd.read_csv(path + "/testing.csv", header=None)

training_df = training_df.iloc[:,:22]
testing_df = training_df.iloc[:,:22]

pca_20 = PCA(n_components=20)
pca_20_training = pd.DataFrame(pca_20.fit_transform(training_df))
pca_20_testing = pd.DataFrame(pca_20.transform(testing_df))
print("PCA 20 explained variance: " + str(np.sum(pca_20.explained_variance_ratio_)))
pca_15 = PCA(n_components=15)
pca_15_training = pd.DataFrame(pca_15.fit_transform(training_df))
pca_15_testing = pd.DataFrame(pca_15.transform(testing_df))
print("PCA 15 explained variance: " + str(np.sum(pca_15.explained_variance_ratio_)))
pca_10 = PCA(n_components=10)
pca_10_training = pd.DataFrame(pca_10.fit_transform(training_df))
pca_10_testing = pd.DataFrame(pca_10.transform(testing_df))
print("PCA 10 explained variance: " + str(np.sum(pca_10.explained_variance_ratio_)))
pca_5 = PCA(n_components=5)
pca_5_training = pd.DataFrame(pca_5.fit_transform(training_df))
pca_5_testing = pd.DataFrame(pca_5.transform(testing_df))
print("PCA 5 explained variance: " + str(np.sum(pca_5.explained_variance_ratio_)))

pca_20_training = pd.concat([pca_20_training, training_df.iloc[:,22:]], axis=1)
pca_20_testing = pd.concat([pca_20_testing, testing_df.iloc[:,22:]], axis=1)
pca_15_training = pd.concat([pca_15_training, training_df.iloc[:,22:]], axis=1)
pca_15_testing = pd.concat([pca_15_testing, testing_df.iloc[:,22:]], axis=1)
pca_10_training = pd.concat([pca_10_training, training_df.iloc[:,22:]], axis=1)
pca_10_testing = pd.concat([pca_10_testing, testing_df.iloc[:,22:]], axis=1)
pca_5_training = pd.concat([pca_5_training, training_df.iloc[:,22:]], axis=1)
pca_5_testing = pd.concat([pca_5_testing, testing_df.iloc[:,22:]], axis=1)

os.makedirs(path + "/pca/20", exist_ok=True)
os.makedirs(path + "/pca/20", exist_ok=True)
os.makedirs(path + "/pca/15", exist_ok=True)
os.makedirs(path + "/pca/15", exist_ok=True)
os.makedirs(path + "/pca/10", exist_ok=True)
os.makedirs(path + "/pca/10", exist_ok=True)
os.makedirs(path + "/pca/5", exist_ok=True)
os.makedirs(path + "/pca/5", exist_ok=True)

pca_20_training.to_csv(path + "/pca/20/training.csv", header=False, index=False)
pca_20_training.to_csv(path + "/pca/20/testing.csv", header=False, index=False)
pca_15_training.to_csv(path + "/pca/15/training.csv", header=False, index=False)
pca_15_training.to_csv(path + "/pca/15/testing.csv", header=False, index=False)
pca_10_training.to_csv(path + "/pca/10/training.csv", header=False, index=False)
pca_10_training.to_csv(path + "/pca/10/testing.csv", header=False, index=False)
pca_5_training.to_csv(path + "/pca/5/training.csv", header=False, index=False)
pca_5_training.to_csv(path + "/pca/5/testing.csv", header=False, index=False)

