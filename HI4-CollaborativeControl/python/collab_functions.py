import numpy as np

def multi_agent_ode(x, t, agent, n):
    """Formulate the differential equation for the whole multi-agent system
    
        dx/dt = multi_agent_ode(x, t, agents, n)
    
        Inputs:
            x - State of the whole multi agent system
            t - current time
            agents - list of all agent specifications
            n - number of states in each agent (must be equal among all agents)
            
        Outputs
           dx/dt - derivative of the state-vector for the multi agent system.
    """
    dxdt = []
    xr = x.reshape((-1, n))
    for a_i, xi in zip(agent, xr):
        y = a_i['h'](x, a_i['measpar'])
        u = a_i['g'](y, a_i['xref'](t), a_i['ctrlpar'])
        dxi_dt = a_i['f'](t, xi, u, a_i['mdlpar'])
        dxdt.append(dxi_dt)
    return np.array(dxdt).reshape((-1))

def CreateAgent(f, mdlpar, g, ctrlpar, h, measpar, xref):
    """Create an agent
    
    Returns a dictionary collecitng all information needed to integrate the behavior of an agent.
    
    Inputs:
        f - Dynamic function for the agent: f(t, x, u, mdlpar)
        mdlpar - Dictionary with model parameters used by f
        g - Control function: g(y, xref, ctrlpar)
        ctrlpar - Dictionary with control parameters used by g
        h - Measurement function: h(x, measpar)
        measpar - Dictionary with parameters used by h
        xref - Reference specification
    """
    return {'f': f, 'mdlpar': mdlpar, 'g': g, 'ctrlpar': ctrlpar, 'h': h, 'measpar': measpar, 'xref': xref}


agent_idx = [range(4*i, 4*(i+1)) for i in range(0, 8)]

measurement_graph = [
    agent_idx[0],  # Agent 1
    np.dstack((np.array([agent_idx[1], agent_idx[0]]), # Agent 2
               np.array([agent_idx[1], agent_idx[2]]))),
    np.dstack((np.array([agent_idx[2], agent_idx[1]]), # Agent 3
               np.array([agent_idx[2], agent_idx[3]]))),
    np.dstack((np.array([agent_idx[3], agent_idx[2]]), # Agent 4
               np.array([agent_idx[3], agent_idx[4]]))),
    np.dstack((np.array([agent_idx[4], agent_idx[3]]), # Agent 5
               np.array([agent_idx[4], agent_idx[5]]))),
    np.dstack((np.array([agent_idx[5], agent_idx[4]]), # Agent 6
               np.array([agent_idx[5], agent_idx[6]]))),
    np.dstack((np.array([agent_idx[6], agent_idx[5]]), # Agent 7
               np.array([agent_idx[6], agent_idx[7]]))),
    np.dstack((np.array([agent_idx[7], agent_idx[6]]),)) # Agent 8
]

formation_measurement_index = [{'idx': mi} for mi in measurement_graph]
formation_references = [lambda t: np.array([0, 4, 0, 0]).reshape(4, -1),    
                        lambda t: np.array([[0, 0], [1, -1], [0, 0], [0, 0]]),
                        lambda t: np.array([[0, 0], [1, -1], [0, 0], [0, 0]]),
                        lambda t: np.array([[0, 0], [1, -1], [0, 0], [0, 0]]),
                        lambda t: np.array([[0, 0], [1, -1], [0, 0], [0, 0]]),
                        lambda t: np.array([[0, 0], [1, -1], [0, 0], [0, 0]]),
                        lambda t: np.array([[0, 0], [1, -1], [0, 0], [0, 0]]),
                        lambda t: np.array([0, 1, 0, 0]).reshape(4, -1)]
