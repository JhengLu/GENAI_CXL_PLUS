import pandas as pd

# Provided text
data_text = """

Finished training it 100/1000 of epoch 0, 26.25 ms/it, loss 0.083371
Finished training it 200/1000 of epoch 0, 17.77 ms/it, loss 0.083515
Finished training it 300/1000 of epoch 0, 18.30 ms/it, loss 0.083672
Finished training it 400/1000 of epoch 0, 17.44 ms/it, loss 0.082935
Finished training it 500/1000 of epoch 0, 19.26 ms/it, loss 0.083580
Finished training it 600/1000 of epoch 0, 17.51 ms/it, loss 0.083138
Finished training it 700/1000 of epoch 0, 17.48 ms/it, loss 0.083406
Finished training it 800/1000 of epoch 0, 17.50 ms/it, loss 0.083437
Finished training it 900/1000 of epoch 0, 21.41 ms/it, loss 0.083266
Finished training it 1000/1000 of epoch 0, 17.43 ms/it, loss 0.083328
Finished training it 100/1000 of epoch 1, 17.46 ms/it, loss 0.083204
Finished training it 200/1000 of epoch 1, 17.50 ms/it, loss 0.083497
Finished training it 300/1000 of epoch 1, 17.49 ms/it, loss 0.083667
Finished training it 400/1000 of epoch 1, 17.47 ms/it, loss 0.082929
Finished training it 500/1000 of epoch 1, 17.49 ms/it, loss 0.083574
Finished training it 600/1000 of epoch 1, 17.46 ms/it, loss 0.083133
Finished training it 700/1000 of epoch 1, 25.84 ms/it, loss 0.083400
Finished training it 800/1000 of epoch 1, 17.44 ms/it, loss 0.083430
Finished training it 900/1000 of epoch 1, 17.44 ms/it, loss 0.083260
Finished training it 1000/1000 of epoch 1, 17.42 ms/it, loss 0.083323
Finished training it 100/1000 of epoch 2, 17.44 ms/it, loss 0.083198
Finished training it 200/1000 of epoch 2, 17.41 ms/it, loss 0.083492
Finished training it 300/1000 of epoch 2, 17.43 ms/it, loss 0.083662
Finished training it 400/1000 of epoch 2, 17.45 ms/it, loss 0.082924
Finished training it 500/1000 of epoch 2, 17.46 ms/it, loss 0.083569
Finished training it 600/1000 of epoch 2, 17.43 ms/it, loss 0.083130
Finished training it 700/1000 of epoch 2, 17.42 ms/it, loss 0.083395
Finished training it 800/1000 of epoch 2, 17.40 ms/it, loss 0.083424
Finished training it 900/1000 of epoch 2, 17.42 ms/it, loss 0.083256
Finished training it 1000/1000 of epoch 2, 17.46 ms/it, loss 0.083320
Finished training it 100/1000 of epoch 3, 17.43 ms/it, loss 0.083194
Finished training it 200/1000 of epoch 3, 17.42 ms/it, loss 0.083488
Finished training it 300/1000 of epoch 3, 34.45 ms/it, loss 0.083659
Finished training it 400/1000 of epoch 3, 17.42 ms/it, loss 0.082921
Finished training it 500/1000 of epoch 3, 17.42 ms/it, loss 0.083565
Finished training it 600/1000 of epoch 3, 17.47 ms/it, loss 0.083127
Finished training it 700/1000 of epoch 3, 17.48 ms/it, loss 0.083390
Finished training it 800/1000 of epoch 3, 17.47 ms/it, loss 0.083420
Finished training it 900/1000 of epoch 3, 17.48 ms/it, loss 0.083253
Finished training it 1000/1000 of epoch 3, 17.43 ms/it, loss 0.083317
Finished training it 100/1000 of epoch 4, 17.48 ms/it, loss 0.083191
Finished training it 200/1000 of epoch 4, 17.48 ms/it, loss 0.083485
Finished training it 300/1000 of epoch 4, 17.47 ms/it, loss 0.083656
Finished training it 400/1000 of epoch 4, 17.44 ms/it, loss 0.082918
Finished training it 500/1000 of epoch 4, 17.49 ms/it, loss 0.083561
Finished training it 600/1000 of epoch 4, 17.47 ms/it, loss 0.083125
Finished training it 700/1000 of epoch 4, 17.46 ms/it, loss 0.083387
Finished training it 800/1000 of epoch 4, 17.48 ms/it, loss 0.083416
Finished training it 900/1000 of epoch 4, 17.49 ms/it, loss 0.083250
Finished training it 1000/1000 of epoch 4, 17.49 ms/it, loss 0.083314
Finished training it 100/1000 of epoch 5, 17.46 ms/it, loss 0.083189
Finished training it 200/1000 of epoch 5, 17.46 ms/it, loss 0.083483
Finished training it 300/1000 of epoch 5, 17.50 ms/it, loss 0.083654
Finished training it 400/1000 of epoch 5, 17.50 ms/it, loss 0.082916
Finished training it 500/1000 of epoch 5, 17.49 ms/it, loss 0.083559
Finished training it 600/1000 of epoch 5, 17.46 ms/it, loss 0.083123
Finished training it 700/1000 of epoch 5, 17.49 ms/it, loss 0.083384
Finished training it 800/1000 of epoch 5, 17.43 ms/it, loss 0.083413
Finished training it 900/1000 of epoch 5, 17.49 ms/it, loss 0.083248
Finished training it 1000/1000 of epoch 5, 17.50 ms/it, loss 0.083312
Finished training it 100/1000 of epoch 6, 17.48 ms/it, loss 0.083186
Finished training it 200/1000 of epoch 6, 17.50 ms/it, loss 0.083481
Finished training it 300/1000 of epoch 6, 17.46 ms/it, loss 0.083652
Finished training it 400/1000 of epoch 6, 17.46 ms/it, loss 0.082914
Finished training it 500/1000 of epoch 6, 17.47 ms/it, loss 0.083557
Finished training it 600/1000 of epoch 6, 52.02 ms/it, loss 0.083122
Finished training it 700/1000 of epoch 6, 17.51 ms/it, loss 0.083381
Finished training it 800/1000 of epoch 6, 17.47 ms/it, loss 0.083411
Finished training it 900/1000 of epoch 6, 17.50 ms/it, loss 0.083246
Finished training it 1000/1000 of epoch 6, 17.46 ms/it, loss 0.083311
Finished training it 100/1000 of epoch 7, 17.49 ms/it, loss 0.083185
Finished training it 200/1000 of epoch 7, 17.51 ms/it, loss 0.083479
Finished training it 300/1000 of epoch 7, 17.47 ms/it, loss 0.083651
Finished training it 400/1000 of epoch 7, 17.47 ms/it, loss 0.082912
Finished training it 500/1000 of epoch 7, 17.48 ms/it, loss 0.083555
Finished training it 600/1000 of epoch 7, 17.50 ms/it, loss 0.083120
Finished training it 700/1000 of epoch 7, 17.48 ms/it, loss 0.083379
Finished training it 800/1000 of epoch 7, 17.51 ms/it, loss 0.083409
Finished training it 900/1000 of epoch 7, 17.51 ms/it, loss 0.083245
Finished training it 1000/1000 of epoch 7, 17.51 ms/it, loss 0.083309
Finished training it 100/1000 of epoch 8, 17.48 ms/it, loss 0.083183
Finished training it 200/1000 of epoch 8, 17.45 ms/it, loss 0.083478
Finished training it 300/1000 of epoch 8, 17.48 ms/it, loss 0.083650
Finished training it 400/1000 of epoch 8, 17.52 ms/it, loss 0.082911
Finished training it 500/1000 of epoch 8, 17.50 ms/it, loss 0.083553
Finished training it 600/1000 of epoch 8, 17.51 ms/it, loss 0.083119
Finished training it 700/1000 of epoch 8, 17.48 ms/it, loss 0.083378
Finished training it 800/1000 of epoch 8, 17.48 ms/it, loss 0.083407
Finished training it 900/1000 of epoch 8, 17.52 ms/it, loss 0.083244
Finished training it 1000/1000 of epoch 8, 17.54 ms/it, loss 0.083308
Finished training it 100/1000 of epoch 9, 17.52 ms/it, loss 0.083182
Finished training it 200/1000 of epoch 9, 17.52 ms/it, loss 0.083476
Finished training it 300/1000 of epoch 9, 17.50 ms/it, loss 0.083649
Finished training it 400/1000 of epoch 9, 17.47 ms/it, loss 0.082909
Finished training it 500/1000 of epoch 9, 17.52 ms/it, loss 0.083552
Finished training it 600/1000 of epoch 9, 17.57 ms/it, loss 0.083118
Finished training it 700/1000 of epoch 9, 17.52 ms/it, loss 0.083376
Finished training it 800/1000 of epoch 9, 17.53 ms/it, loss 0.083405
Finished training it 900/1000 of epoch 9, 17.54 ms/it, loss 0.083242
Finished training it 1000/1000 of epoch 9, 17.48 ms/it, loss 0.083307


"""

# Extract ms/it data
ms_per_it = [float(line.split(",")[1].split(" ")[1]) for line in data_text.strip().split("\n")]

# Create a DataFrame
df = pd.DataFrame({"Epoch": [i // 100 + 1 for i in range(len(ms_per_it))], "Iteration": [100 * ((i % 100) + 1) for i in range(len(ms_per_it))], "ms/it": ms_per_it})

# Save to Excel
df.to_excel("ms_per_iteration_data_new.xlsx", index=False)
