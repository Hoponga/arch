
V = [1000] 
start = 1000 
A = 10000
f = 1
current = 1000 
prev_output = current 
feedback = 1
for i in range(100): 
    expected_output = A*(current - f*V[i])
    V.append(expected_output/A) 



import matplotlib.pyplot as plt
plt.plot(V)
plt.savefig("i love eecs16a.png")