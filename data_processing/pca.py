import pandas as pd
from sklearn.decomposition import PCA
import numpy as np
import os

# Gets path to data and reads csv files to dataframes
path=input("Enter path to folder with split data\n")
training_df = pd.read_csv(path + "/training.csv", header=None)
validation_df = pd.read_csv(path + "/validation.csv", header=None)
testing_df = pd.read_csv(path + "/testing.csv", header=None)

# Splits dataframes to inputs and outputs
input_training_df = training_df.iloc[:,:22]
input_validation_df = validation_df.iloc[:,:22]
input_testing_df = testing_df.iloc[:,:22]
output_training_df = training_df.iloc[:,22:]
output_validation_df = validation_df.iloc[:,22:]
output_testing_df = testing_df.iloc[:,22:]

# Performs PCA with 20 components using fit_transform for training data and transform for
# other, prints explained variance to terminal
pca_20 = PCA(n_components=20)
pca_20_training = pd.DataFrame(pca_20.fit_transform(input_training_df))
pca_20_validation = pd.DataFrame(pca_20.transform(input_validation_df))
pca_20_testing = pd.DataFrame(pca_20.transform(input_testing_df))
print("PCA 20 explained variance: " + str(np.sum(pca_20.explained_variance_ratio_)))

# Performs PCA with 15 components using fit_transform for training data and transform for
# other, prints explained variance to terminal
pca_15 = PCA(n_components=15)
pca_15_training = pd.DataFrame(pca_15.fit_transform(input_training_df))
pca_15_validation = pd.DataFrame(pca_15.transform(input_validation_df))
pca_15_testing = pd.DataFrame(pca_15.transform(input_testing_df))
print("PCA 15 explained variance: " + str(np.sum(pca_15.explained_variance_ratio_)))

# Performs PCA with 10 components using fit_transform for training data and transform for
# other, prints explained variance to terminal
pca_10 = PCA(n_components=10)
pca_10_training = pd.DataFrame(pca_10.fit_transform(input_training_df))
pca_10_validation = pd.DataFrame(pca_10.transform(input_validation_df))
pca_10_testing = pd.DataFrame(pca_10.transform(input_testing_df))
print("PCA 10 explained variance: " + str(np.sum(pca_10.explained_variance_ratio_)))

# Performs PCA with 5 components using fit_transform for training data and transform for
# other, prints explained variance to terminal
pca_5 = PCA(n_components=5)
pca_5_training = pd.DataFrame(pca_5.fit_transform(input_training_df))
pca_5_validation = pd.DataFrame(pca_5.transform(input_validation_df))
pca_5_testing = pd.DataFrame(pca_5.transform(input_testing_df))
print("PCA 5 explained variance: " + str(np.sum(pca_5.explained_variance_ratio_)))

# Concatenates 20 PCA components to outputs
pca_20_training = pd.concat([pca_20_training, output_training_df], axis=1)
pca_20_validation = pd.concat([pca_20_validation, output_validation_df], axis=1)
pca_20_testing = pd.concat([pca_20_testing, output_testing_df], axis=1)

# Concatenates 15 PCA components to outputs
pca_15_training = pd.concat([pca_15_training, output_training_df], axis=1)
pca_15_validation = pd.concat([pca_15_validation, output_validation_df], axis=1)
pca_15_testing = pd.concat([pca_15_testing, output_testing_df], axis=1)

# Concatenates 10 PCA components to outputs
pca_10_training = pd.concat([pca_10_training, output_training_df], axis=1)
pca_10_validation = pd.concat([pca_10_validation, output_validation_df], axis=1)
pca_10_testing = pd.concat([pca_10_testing, output_testing_df], axis=1)

# Concatenates 5 PCA components to outputs
pca_5_training = pd.concat([pca_5_training, output_training_df], axis=1)
pca_5_validation = pd.concat([pca_5_validation, output_validation_df], axis=1)
pca_5_testing = pd.concat([pca_5_testing, output_testing_df], axis=1)

# Creates directories to write outputs
os.makedirs(path + "/pca/20", exist_ok=True)
os.makedirs(path + "/pca/15", exist_ok=True)
os.makedirs(path + "/pca/10", exist_ok=True)
os.makedirs(path + "/pca/5", exist_ok=True)

# Writes data to csv files
pca_20_training.to_csv(path + "/pca/20/training.csv", header=False, index=False)
pca_20_validation.to_csv(path + "/pca/20/validation.csv", header=False, index=False)
pca_20_testing.to_csv(path + "/pca/20/testing.csv", header=False, index=False)
pca_15_training.to_csv(path + "/pca/15/training.csv", header=False, index=False)
pca_15_validation.to_csv(path + "/pca/15/validation.csv", header=False, index=False)
pca_15_testing.to_csv(path + "/pca/15/testing.csv", header=False, index=False)
pca_10_training.to_csv(path + "/pca/10/training.csv", header=False, index=False)
pca_10_validation.to_csv(path + "/pca/10/validation.csv", header=False, index=False)
pca_10_testing.to_csv(path + "/pca/10/testing.csv", header=False, index=False)
pca_5_training.to_csv(path + "/pca/5/training.csv", header=False, index=False)
pca_5_validation.to_csv(path + "/pca/5/validation.csv", header=False, index=False)
pca_5_testing.to_csv(path + "/pca/5/testing.csv", header=False, index=False)