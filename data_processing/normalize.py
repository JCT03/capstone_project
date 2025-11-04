import pandas as pd
from sklearn.preprocessing import StandardScaler
from sklearn.preprocessing import MinMaxScaler
import os

path=input("Enter path to folder with split data\n")
training_df = pd.read_csv(path + "/training.csv", header=None)
validation_df = pd.read_csv(path + "/validation.csv", header=None)
testing_df = pd.read_csv(path + "/testing.csv", header=None)

z_scaler = StandardScaler()
min_max_scaler = MinMaxScaler()

min_max_normalized_training = pd.DataFrame(min_max_scaler.fit_transform(training_df))
min_max_normalized_validation = pd.DataFrame(min_max_scaler.transform(validation_df))
min_max_normalized_testing = pd.DataFrame(min_max_scaler.transform(testing_df))

z_normalized_training = pd.DataFrame(z_scaler.fit_transform(training_df))
z_normalized_validation = pd.DataFrame(z_scaler.transform(validation_df))
z_normalized_testing = pd.DataFrame(z_scaler.transform(testing_df))

os.makedirs(path + "/min_max", exist_ok=True)
os.makedirs(path + "/z", exist_ok=True)

min_max_normalized_training.to_csv(path + "/min_max/training.csv", header=False, index=False)
min_max_normalized_validation.to_csv(path + "/min_max/validation.csv", header=False, index=False)
min_max_normalized_testing.to_csv(path + "/min_max/testing.csv", header=False, index=False)
z_normalized_training.to_csv(path + "/z/training.csv", header=False, index=False)
z_normalized_validation.to_csv(path + "/z/validation.csv", header=False, index=False)
z_normalized_testing.to_csv(path + "/z/testing.csv", header=False, index=False)