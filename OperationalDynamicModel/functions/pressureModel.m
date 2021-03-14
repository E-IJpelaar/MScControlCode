function dpdt = pressureModel(t,p,V,tau)

dpdt = -(1/tau)*p + (11.2/tau)*V;