from miniflow import *

x, y, z = Input(), Input(), Input()

f = Add(x, y, z)
g = Mul(x, y, z)

feed_dict = {x: 10, y: 5, z: 1}

sorted_neurons = topological_sort(feed_dict)
output1 = forward_pass(f, sorted_neurons)
output2 = forward_pass(g, sorted_neurons)


print("{} + {} + {} = {} (according to miniflow)".format(feed_dict[x],
feed_dict[y], feed_dict[z], output1))

print("{} * {} * {} = {} (according to miniflow)".format(feed_dict[x],
feed_dict[y], feed_dict[z], output2))
