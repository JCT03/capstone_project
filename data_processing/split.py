import pandas as pd

# Gets path to data
path = input("Enter path to all.csv\n")

# Reads data into dataframe
df = pd.read_csv(path, header=None)

# Randomizes order of dataframe
df = df.sample(frac=1)

# Puts first 80% in training dataframe
training_df = df.iloc[:int(df.shape[0]*.8)]

# Puts next 10% in validation dataframe
validation_df = df.iloc[int(df.shape[0]*.8):int(df.shape[0]*.9)]

# Puts last 10% in testing dataframe
testing_df = df.iloc[int(df.shape[0]*.9):]

# Writes dataframes to csv files
training_df.to_csv(path[:-7] + "training.csv", header=False, index=False)
validation_df.to_csv(path[:-7] + "validation.csv", header=False, index=False)
testing_df.to_csv(path[:-7] + "testing.csv", header=False, index=False)