import pandas as pd
import matplotlib.pyplot as plt

ee = pd.read_csv("ee_from_tf.csv")
ideal = pd.read_csv("ideal.csv")

print("ee_from_tf.csv shape:", ee.shape)
print("Primeras filas ee_from_tf.csv:")
print(ee[['x', 'y']].head())

x_exec = ee['x'].astype(float).to_numpy()
y_exec = ee['y'].astype(float).to_numpy()

print("Length executed:", len(x_exec), len(y_exec))
print("Valores ejecutados (primeros 5):", x_exec[:5], y_exec[:5])


print("Primeros 5 valores de x_exec:", x_exec[:5])
print("Primeros 5 valores de y_exec:", y_exec[:5])
print("¿Algún valor NaN en x_exec?", pd.isna(x_exec).any())
print("¿Algún valor NaN en y_exec?", pd.isna(y_exec).any())


x_ideal = []
y_ideal = []
for i in range(201):
    x_col = f'field.poses{i}.position.x'
    y_col = f'field.poses{i}.position.y'
    if x_col in ideal.columns and y_col in ideal.columns:
        x_ideal.append(float(ideal[x_col][0]))
        y_ideal.append(float(ideal[y_col][0]))

print("Length ideal:", len(x_ideal), len(y_ideal))

plt.figure(figsize=(8,6))
if len(x_exec) > 0:
    plt.plot(x_exec, y_exec, label='Ejecutada', color='red')
else:
    print("Trayectoria ejecutada vacía!")

plt.plot(x_ideal, y_ideal, label='Deseada', linestyle='--', color='blue')

plt.xlabel("X")
plt.ylabel("Y")
plt.title("Comparación de trayectorias")
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()
