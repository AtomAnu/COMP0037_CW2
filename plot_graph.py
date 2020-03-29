import matplotlib.pyplot as plt

baseline_file = open('entropy_data_baseline.txt','r')
baseline_contents = baseline_file.readlines()
#print(baseline_contents)
baseline_time_steps = [i for i in range(len(baseline_contents))]
#print(baseline_time_steps)
baseline_file.close()

file = open('entropy_data.txt','r')
contents = file.readlines()
print(contents)
time_steps = [i for i in range(len(contents))]
file.close()

#plt.plot(time_steps, contents, label='Implemented')
#plt.plot(baseline_time_steps, baseline_contents, label='Baseline')
plt.plot(baseline_time_steps, baseline_contents)
#plt.legend()
plt.show()