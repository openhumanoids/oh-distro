import scipy.io
from gurobipy import *

mat=scipy.io.loadmat('model_t_0.100.mat')

# Create a new model
m=Model("QP control")

Q=mat['Q']
c=mat['c'][0]
lb=mat['lb'].transpose()[0]
ub=mat['ub'].transpose()[0]
Aeq=mat['Aeq']
beq=mat['beq'].transpose()[0]
Aineq=mat['Ain']
bineq=mat['bin'].transpose()[0]


nvar = len(lb)

variables = dict()
for i in xrange(nvar):
	# Create variables
	variables["x"+str(i)] = m.addVar(lb=lb[i],ub=ub[i], vtype=GRB.CONTINUOUS, name="x"+str(i))

# Integrate new variables
m.update()

# Set objective
var = variables.values();

expr = 0
for i in xrange(nvar):
	for j in xrange(nvar):
		expr += Q[i][j]*var[i]*var[j]

for i in xrange(nvar):
	expr += c[i]*var[i]

m.setObjective(expr, GRB.MINIMIZE)

for i in range(Aeq.shape[0]):
	lhs = 0
	for j in range(Aeq.shape[1]):
		lhs += Aeq[i][j]*var[j]
	m.addConstr(lhs == beq[i], "eq"+str(i))

for i in range(Aineq.shape[0]):
	lhs = 0
	for j in range(Aineq.shape[1]):
		lhs += Aineq[i][j]*var[j]
	m.addConstr(lhs <= bineq[i], "ineq"+str(i))



m.Params.TuneTimeLimit=500
#m.optimize()
m.tune()

for i in range(m.tuneResultCount):
	m.getTuneResult(i)
	m.write('tune'+str(i)+'.prm')


