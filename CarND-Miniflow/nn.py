from miniflow import *


inputs, weights, bias, ideal_output = Input(), Input(), Input(), Input()

x = np.array([[-1., -2.], [-1, -2]])
w = np.array([[2., -3], [2., -3]])
b = np.array([-3., -5])
a = np.array([[1.2e-4, 9.8e-1], [1.2e-4, 9.8e-1]])

feed_dict = {inputs: x, weights: w, bias: b, ideal_output: a}

f = Linear(inputs, weights, bias)
g = Sigmoid(f)
cost = MSE(g, ideal_output)

graph = topological_sort(feed_dict)
output = forward_pass(cost, graph)

print("linear output (according to miniflow) is: {}".format(output))
print("MSE Cost is: ", output)
