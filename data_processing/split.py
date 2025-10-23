import pandas as pd
path = input("Enter path to all.csv\n")
df = pd.read_csv(path, header=None)
df = df.sample(frac=1)
training_df = df.iloc[:int(df.shape[0]*.8)]
testing_df = df.iloc[int(df.shape[0]*.8):]
training_df.to_csv(path[:-7] + "training.csv", header=False, index=False)
testing_df.to_csv(path[:-7] + "testing.csv", header=False, index=False)