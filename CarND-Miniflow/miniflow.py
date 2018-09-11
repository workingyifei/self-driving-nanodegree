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


class Add(Neuron):
	def __init__(self, *inputs):
		Neuron.__init__(self, inputs)

	def forward(self):
		self.value = 0
		for i in range(len(self.inbound_neurons)):
			self.value += self.inbound_neurons[i].value

class Mul(Neuron):
	def __init__(self, *inputs):
		Neuron.__init__(self, inputs)

	def forward(self):
		self.value = 1 
		for i in range(len(self.inbound_neurons)):
			self.value = self.value * self.inbound_neurons[i].value

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
