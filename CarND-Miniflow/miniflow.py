import numpy as np

class Neuron():
	def __init__(self, inbound_neurons=[]):
		self.inbound_neurons = inbound_neurons
		self.outbound_neurons = []
		for n in self.inbound_neurons:
			n.outbound_neurons.append(self)

		self.value = None

	def forward(self):

		# perform forward propagation

		if value is not None:
			self.value = value

class Input(Neuron):
	def __init__(self):
		Neuron.__init__(self)

	def forward(self, value=None):
		if value:
		 	self.value = value


class Linear(Neuron):
	def __init__(self, inputs, weights, bias):
		Neuron.__init__(self, [inputs, weights, bias])

	def forward(self):
		inputs = self.inbound_neurons[0].value
		weights = self.inbound_neurons[1].value
		bias = self.inbound_neurons[2].value

		self.value = np.dot(inputs, weights) + np.array([bias, bias])


class Sigmoid(Neuron):
	def __init__(self, neuron):
		Neuron.__init__(self, [neuron])

	def sigmoid(self, x):
		return 1. / (1. + np.exp(-x))

	def forward(self):
		self.value = self.sigmoid(self.inbound_neurons[0].value)

def topological_sort(feed_dict):
	input_neurons = [n for n in feed_dict.keys()]
	G = {}
	neurons = [n for n in input_neurons]
	while len(neurons) > 0:
	    n = neurons.pop(0)
	    if n not in G:
	        G[n] = {'in': set(), 'out': set()}
	    for m in n.outbound_neurons:
	        if m not in G:
	            G[m] = {'in': set(), 'out': set()}
	        G[n]['out'].add(m)
	        G[m]['in'].add(n)
	        neurons.append(m)

	L = []
	S = set(input_neurons)
	while len(S) > 0:
	    n = S.pop()

	    if isinstance(n, Input):
	        n.value = feed_dict[n]

	    L.append(n)
	    for m in n.outbound_neurons:
	        G[n]['out'].remove(m)
	        G[m]['in'].remove(n)
	        # if no other incoming edges add to S
	        if len(G[m]['in']) == 0:
	            S.add(m)
	return L


def forward_pass(output_neuron, sorted_neurons):

    for n in sorted_neurons:
        n.forward()

    return output_neuron.value


