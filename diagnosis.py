from cnf import *
from ortools.sat.python import cp_model

objects = ['Outlet', 'Rasp-Pi', 'Power-Board',
           'Arduino', 'Sensor-Board0', 'Sensor-Board1']
actuators = ['Fans', 'LEDs', 'Pump']
sensors = ['H-T0', 'Light0', 'Moisture0', 'H-T1', 'Light1', 'Moisture1',
           'Wlevel']
relations = ['working', 'connected', 'powered', 'signal', 'expected-result']

def powered(comp): return 'powered(%s)' %comp
def working(comp): return 'working(%s)' %comp
def connected(from_comp, to_comp):
    return 'connected(%s, %s)' %(from_comp, to_comp)
def signal(signal, component): return 'signal(%s, %s)' %(signal, component)
def rasp_pi_signal(the_signal): return signal(the_signal, 'Rasp-Pi')
def expected_result(actuator): return 'expected-result(%s)' %actuator

def create_relation(name, model, variables):
    variables[name] = model.NewBoolVar(name)

def create_relations(relations, model, variables):
    for relation in relations: create_relation(relation, model, variables)

def create_working_relations(model, variables):
    create_relations([working(comp) for comp in objects + actuators + sensors],
                     model, variables)

def create_connected_relations(model, variables):
    # BEGIN STUDENT CODE
    con_list = [('Power-Board', 'Fans'), ('Power-Board', 'Pump'), ('Power-Board', 'LEDs'), ('Outlet', 'Power-Board'), ('Outlet', 'Rasp-Pi'), 
    ('Arduino', 'Rasp-Pi'), ('Rasp-Pi', 'Arduino'), ('Arduino', 'Power-Board'), ('Sensor-Board0', 'Arduino'), ('Wlevel', 'Arduino'), ('Sensor-Board1', 'Arduino'),
    ('H-T0', 'Sensor-Board0'), ('Light0', 'Sensor-Board0'), ('Moisture0', 'Sensor-Board0'),
    ('H-T1', 'Sensor-Board1'), ('Light1', 'Sensor-Board1'), ('Moisture1', 'Sensor-Board1')] 
    create_relations([connected(comp0, comp1) for comp0, comp1 in con_list],
                     model, variables)
    # END STUDENT CODE
    pass

def create_powered_relations(model, variables):
    # BEGIN STUDENT CODE
    power_list = actuators + ['Outlet', 'Rasp-Pi', 'Power-Board']
    create_relations([powered(comp) for comp in power_list],
                     model, variables)
    # END STUDENT CODE
    pass

def create_signal_relations(model, variables):
    # BEGIN STUDENT CODE
    sensor_gen = [(sensor, sensor) for sensor in sensors]
    sb0_rec = [(sensor, 'Sensor-Board0') for sensor in ['H-T0', 'Light0', 'Moisture0']]
    sb1_rec = [(sensor, 'Sensor-Board1') for sensor in ['H-T1', 'Light1', 'Moisture1']]
    Arduino_rec_s = [(sensor, 'Arduino') for sensor in sensors]
    RaspPi_rec = [(sensor, 'Rasp-Pi') for sensor in sensors]
    RaspPi_gen = [(actuator, 'Rasp-Pi') for actuator in actuators]
    Arduino_rec_a = [(actuator, 'Arduino') for actuator in actuators]
    PB_rec_a = [(actuator, 'Power-Board') for actuator in actuators]
    signal_list = sensor_gen + sb0_rec + sb1_rec + Arduino_rec_s + RaspPi_rec + RaspPi_gen + Arduino_rec_a + PB_rec_a
    create_relations([signal(sig, comp) for sig, comp in signal_list],
                     model, variables)
    # END STUDENT CODE
    pass

def create_expected_result_relations(model, variables):
    # BEGIN STUDENT CODE
    create_relations([expected_result(comp) for comp in actuators],
                    model, variables)
    # END STUDENT CODE
    pass

def create_relation_variables(model):
    variables = {}
    create_working_relations(model, variables)
    create_connected_relations(model, variables)
    create_powered_relations(model, variables)
    create_signal_relations(model, variables)
    create_expected_result_relations(model, variables)
    return variables

def add_constraint_to_model(constraint, model, variables):
    for disj in (eval(constraint) if isinstance(constraint, str) else constraint):
        conv_disj = [variables[lit] if not is_negated(lit) else
                     variables[lit[1]].Not() for lit in disj]
        model.AddBoolOr(conv_disj)

def create_powered_constraint(from_comp, to_comp, model, variables):
    constraint = "IFF('%s', AND('%s', '%s'))" %(powered(to_comp),
                                                connected(from_comp, to_comp),
                                                working(from_comp))
    add_constraint_to_model(constraint, model, variables)

def create_powered_actuator_constraint(actuator, model, variables):
    constraint = ("IFF('%s', AND('%s', AND('%s', AND('%s', '%s'))))"
                  %(powered(actuator), connected('Power-Board', actuator),
                    powered('Power-Board'), working('Power-Board'),
                    signal(actuator, 'Power-Board')))
    add_constraint_to_model(constraint, model, variables)

def create_powered_constraints(model, variables):
    add_constraint_to_model(LIT(powered('Outlet')), model, variables)
    create_powered_constraint('Outlet', 'Rasp-Pi', model, variables)
    create_powered_constraint('Outlet', 'Power-Board', model, variables)
    for actuator in actuators:
        create_powered_actuator_constraint(actuator, model, variables)

def create_signal_constraints(model, variables):
    # BEGIN STUDENT CODE
    for sensor in ['H-T0', 'Light0', 'Moisture0']:
        constraint = "IFF('%s', AND('%s', AND('%s', '%s')))" % (signal(sensor, 'Sensor-Board0'),
                                                                connected(sensor, 'Sensor-Board0'),
                                                                working(sensor),
                                                                signal(sensor, sensor))
        cons_A_s = "IFF('%s', AND('%s', AND('%s', '%s')))" % (signal(sensor, 'Arduino'),
                                                              connected('Sensor-Board0', 'Arduino'),
                                                              working('Sensor-Board0'),
                                                              signal(sensor, 'Sensor-Board0'))                                                         
        add_constraint_to_model(constraint, model, variables)    
        add_constraint_to_model(cons_A_s, model, variables)     
    for sensor in ['H-T1', 'Light1', 'Moisture1']:
        constraint = "IFF('%s', AND('%s', AND('%s', '%s')))" % (signal(sensor, 'Sensor-Board1'),
                                                                connected(sensor, 'Sensor-Board1'),
                                                                working(sensor),
                                                                signal(sensor, sensor))
        cons_A_s = "IFF('%s', AND('%s', AND('%s', '%s')))" % (signal(sensor, 'Arduino'),
                                                              connected('Sensor-Board1', 'Arduino'),
                                                              working('Sensor-Board1'),
                                                              signal(sensor, 'Sensor-Board1'))  
        add_constraint_to_model(constraint, model, variables) 
        add_constraint_to_model(cons_A_s, model, variables)
    constraint = "IFF('%s', AND('%s', AND('%s', '%s')))" % (signal('Wlevel', 'Arduino'),
                                                            connected('Wlevel', 'Arduino'),
                                                            working('Wlevel'),
                                                            signal('Wlevel', 'Wlevel')) 
    add_constraint_to_model(constraint, model, variables)
    for sensor in sensors:
        constraint = "IFF('%s', AND('%s', AND('%s', '%s')))" % (signal(sensor, 'Rasp-Pi'),
                                                                connected('Arduino', 'Rasp-Pi'),
                                                                working('Arduino'),
                                                                signal(sensor, 'Arduino'))
        add_constraint_to_model(constraint, model, variables)                                                                                                
    for actuator in actuators:
        cons_A_a = "IFF('%s', AND('%s', AND('%s', '%s')))" % (signal(actuator, 'Arduino'),
                                                              connected('Rasp-Pi', 'Arduino'),
                                                              working('Rasp-Pi'),
                                                              signal(actuator, 'Rasp-Pi'))              
        cons_P = "IFF('%s', AND('%s', AND('%s', '%s')))" % (signal(actuator, 'Power-Board'),
                                                            connected('Arduino', 'Power-Board'),
                                                            working('Arduino'),
                                                            signal(actuator, 'Arduino'))
        add_constraint_to_model(cons_A_a, model, variables)                
        add_constraint_to_model(cons_P, model, variables)                                                  
    # END STUDENT CODE
    pass

def create_sensor_generation_constraints(model, variables):
    # BEGIN STUDENT CODE
    for sensor in sensors:
        constraint = "IFF('%s', '%s')" %(signal(sensor, sensor),
                                         working(sensor))
        add_constraint_to_model(constraint, model, variables)
    # END STUDENT CODE
    pass

def create_expected_result_constraints(model, variables):
    # BEGIN STUDENT CODE
    constraint1 = "IFF('%s', AND('%s', AND('%s', OR('%s', '%s'))))" %(expected_result('Fans'),
                                                                     powered('Fans'),
                                                                     working('Fans'),
                                                                     signal('H-T0', 'Rasp-Pi'),
                                                                     signal('H-T1', 'Rasp-Pi'))
    add_constraint_to_model(constraint1, model, variables)
    constraint2 = "IFF('%s', AND('%s', AND('%s', OR('%s', '%s'))))" %(expected_result('LEDs'),
                                                                     powered('LEDs'),
                                                                     working('LEDs'),
                                                                     signal('Light0', 'Rasp-Pi'),
                                                                     signal('Light1', 'Rasp-Pi'))
    add_constraint_to_model(constraint2, model, variables)
    constraint3 = "IFF('%s', AND('%s', AND('%s', OR('%s', OR('%s', '%s')))))" %(expected_result('Pump'),
                                                                                powered('Pump'),
                                                                                working('Pump'),
                                                                                signal('Moisture0', 'Rasp-Pi'),
                                                                                signal('Moisture1', 'Rasp-Pi'),
                                                                                signal('Wlevel', 'Rasp-Pi'))
    add_constraint_to_model(constraint3, model, variables)
    # END STUDENT CODE
    pass

def create_constraints(model, variables):
    create_powered_constraints(model, variables)
    create_signal_constraints(model, variables)
    create_sensor_generation_constraints(model, variables)
    create_expected_result_constraints(model, variables)

def create_greenhouse_model():
    model = cp_model.CpModel()
    variables = create_relation_variables(model)
    create_constraints(model, variables)
    return (model, variables)
    
def collect_diagnosis(solver, variables):
    return set([var for var in variables
                if ((var.startswith('connected') or var.startswith('working')) and
                    solver.BooleanValue(variables[var]) == False)])

class DiagnosesCollector(cp_model.CpSolverSolutionCallback):
    def __init__(self, variables):
        cp_model.CpSolverSolutionCallback.__init__(self)
        # BEGIN STUDENT CODE
        self.__variables = variables
        self.diagnoses = []
        # END STUDENT CODE

    def OnSolutionCallback(self):
        # Extract the connected and working relations that are False
        # BEGIN STUDENT CODE 
        d_set = collect_diagnosis(self, self.__variables)
        self.diagnoses.append(d_set)
        return self.diagnoses
        # END STUDENT CODE
        pass

def diagnose(observations):
    model, variables = create_greenhouse_model()
    add_constraint_to_model(observations, model, variables)

    collector = DiagnosesCollector(variables)
    diagnoses = []
    solver = cp_model.CpSolver()
    solver.SearchForAllSolutions(model, collector)
    # Remove all redundant diagnoses (those that are supersets
    #   of other diagnoses).
    # BEGIN STUDENT CODE
    collector.diagnoses.sort(key=len)
    d_list = []
    for diagnosis in collector.diagnoses:
        superset = False
        for d in d_list:
            if diagnosis.issuperset(d):
                superset = True
        if not superset:
           d_list.append(diagnosis)
    diagnoses = d_list
    # END STUDENT CODE

    return diagnoses
