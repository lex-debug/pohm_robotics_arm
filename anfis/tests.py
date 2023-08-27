# import anfis
# import membership.mfDerivs
# import membership.membershipfunction
# import numpy

# ts = numpy.loadtxt("trainingSet.txt", usecols=[1,2,3])#numpy.loadtxt('c:\\Python_fiddling\\myProject\\MF\\trainingSet.txt',usecols=[1,2,3])
# X = ts[:,0:2]
# Y = ts[:,2]

# print(X)
# print(Y)

# mf = [[['gaussmf',{'mean':0.,'sigma':1.}],['gaussmf',{'mean':-1.,'sigma':2.}],['gaussmf',{'mean':-4.,'sigma':10.}],['gaussmf',{'mean':-7.,'sigma':7.}]],
#             [['gaussmf',{'mean':1.,'sigma':2.}],['gaussmf',{'mean':2.,'sigma':3.}],['gaussmf',{'mean':-2.,'sigma':10.}],['gaussmf',{'mean':-10.5,'sigma':5.}]]]


# mfc = membership.membershipfunction.MemFuncs(mf)
# anf = anfis.ANFIS(X, Y, mfc)
# anf.trainHybridJangOffLine(epochs=20)
# print(round(anf.consequents[-1][0],6))
# print(round(anf.consequents[-2][0],6))
# print(round(anf.fittedValues[9][0],6))
# if round(anf.consequents[-1][0],6) == -5.275538 and round(anf.consequents[-2][0],6) == -1.990703 and round(anf.fittedValues[9][0],6) == 0.002249:
# 	print('test is good')

# print("Plotting errors")
# anf.plotErrors()
# print("Plotting results")
# anf.plotResults()
import numpy as np

# We will use the data to train the anfis
# The training data is in the form of input-output pair
# While it can have multiple input, it can only have a single output
ts = np.loadtxt("/home/alexpc/dev_ws/src/pohm_robotics_arm/pohm_robotics_arm/data/data2.txt", usecols=[1,2,3])#numpy.loadtxt('c:\\Python_fiddling\\myProject\\MF\\trainingSet.txt',usecols=[1,2,3])

# Now we need to normalize the input value
# In another word, we are converting the crisp value to the fuzzy value
X = ts[:,0:2]
x0max = np.max(X[:, 0])
x1max = np.max(X[:, 1])
x0min = np.min(X[:, 0])
x1min = np.min(X[:, 1])
X[:, 0] = ((X[:, 0] - x0min) / (x0max - x0min)) * 20.0 -10.0
X[:, 1] = ((X[:, 1] - x1min) / (x1max - x1min)) * 20.0 -10.0

Y = ts[:,2]

a = [[0, 1, 2, 3], [0, 1, 2, 3]]
b = all(len())

