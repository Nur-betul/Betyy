import matplotlib.pyplot as plt
import numpy as np
import functools
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def create_data_model():
    """Problem için verileri oluşturur."""
    data = {}
    
    data['locations'] =  [    # İstenilen locations
            (270, 133), (228, 161), 
            (220, 169), (212, 169), 
            (172, 169), (140, 169), 
            (104, 161), (80, 157), 
            (56, 137), (32, 169), 
            (32, 145), (32, 129), 
            (32, 121), (16, 97), 
            (8, 97), (8, 73), 
            (32, 41)]

    data['numlocations_'] = len(data['locations'])
          
    data['time_windows'] = [
        (0, 25), 
     (35, 60), (50, 75), 
     (25, 70), (25, 65),  
     (0, 25), (25, 50), 
     (0, 50), (25, 50), 
     (0, 25), (50, 80), 
     (50, 75), (0, 25), 
     (25, 50), (35, 60),
     (50, 75), (25, 75)] # 15, 16
    
    
    data['demands'] = \
          [0,  # depot
            1, 1,  # 1, 2
            2, 4,  # 3, 4
            2, 4,  # 5, 6
            8, 8,  # 7, 8
            1, 2,  # 9,10
            1, 2,  # 11,12
            4, 4,  # 13, 14
            8, 8]  # 15, 16
          
    data['time_per_demand_unit'] = 4  # Yük başına tanınan süre
    data['num_vehicles'] = 4
    data['breaks'] = [(13, False), (2, True), (2, True), (32,False)]
    data['vehicle_capacity'] = [30, 15, 15, 30]
    data['vehicle_speed'] = 77  # Travel speed: 5km/h converted in m/min
    # data['depot'] = 0
    data['starts'] = [1, 2, 15, 16]
    """ data['starts'] = [0, 0, 0, 0, 0]"""
    data['ends'] = [0, 0, 0, 0] 
    
    return data



def location_distance(position_1, position_2):
    """İki mesafe arasındaki uzaklığı döndürür"""
    return (abs(position_1[0] - position_2[0]) +
            abs(position_1[1] - position_2[1])
)

def create_distance_evaluator(data):
    """Noktalar arasındaki mesafeyi döndüren callback'i oluşturur"""
    distances_ = {}
    for from_node in range(data['numlocations_']):
        distances_[from_node] = {}
        for to_node in range(data['numlocations_']):
            if from_node == to_node:
                distances_[from_node][to_node] = 0
            else:
                distances_[from_node][to_node] = (location_distance(
                    data['locations'][from_node], data['locations'][to_node]))

    def distance_evaluator(manager, from_node, to_node):
        """İki nodül arasındaki mesafeyi döndürür."""
        return distances_[manager.IndexToNode(from_node)][manager.IndexToNode(to_node)]

    return distance_evaluator


def create_demand_evaluator(data):
    """Lokasyonlardaki talep edilen yük miktarını döndüren callback'i oluşturur."""
    demands_ = data['demands']

    def demand_evaluator(manager, node):
        """Mevcut nodüldeki talepleri döner."""
        return demands_[manager.IndexToNode(node)]

    return demand_evaluator


def add_capacity_constraints(routing, data, demand_evaluator_index):
    """Kapasite kısıtı ekler"""
    capacity = 'Capacity'
    routing.AddDimensionWithVehicleCapacity(
        demand_evaluator_index,
        0,  # null capacity slack
        data['vehicle_capacity'],
        True,  # start cumul to zero
        capacity)


def create_time_evaluator(data):
    """İki konum arasındaki toplam zamanı döndüren callback'i oluşturur."""

    def service_time(data, node):
        """Belirlenen konumlarda yük yüklerkenki süreyi döndürür."""
        return data['demands'][node] * data['time_per_demand_unit']

    def travel_time(data, from_node, to_node):
        """İki konum arasındaki seyahat süresini döndürür"""
        if from_node == to_node:
            travel_time = 0
        else:
            travel_time = location_distance(
                data['locations'][from_node],
                data['locations'][to_node]) / data['vehicle_speed']
        return travel_time

    total_time_ = {}
    for from_node in range(data['numlocations_']):
        total_time_[from_node] = {}
        for to_node in range(data['numlocations_']):
            if from_node == to_node:
                total_time_[from_node][to_node] = 0
            else:
                total_time_[from_node][to_node] = int(
                    service_time(data, from_node) +
                    travel_time(data, from_node, to_node))

    def time_evaluator(manager, from_node, to_node):
        """İki nodül arasındaki total zaman"""
        return total_time_[manager.IndexToNode(from_node)][manager.IndexToNode(
            to_node)]

    return time_evaluator


def add_time_window_constraints(routing, manager, data, time_evaluator_index):
    """Zaman Kısıtı Ekle"""
    time = 'Time'
    horizon = 150
    routing.AddDimension(
        time_evaluator_index,
        horizon,  
        horizon, 
        False,  
        time)
    time_dimension = routing.GetDimensionOrDie(time)
    for location_idx, time_window in enumerate(data['time_windows']):
        #if location_idx == data['depot']:
        if location_idx in data['ends'] or  location_idx in data['starts']:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
        routing.AddToAssignment(time_dimension.SlackVar(index))
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(data['time_windows'][0][0],
                                                data['time_windows'][0][1])
        
        routing.AddToAssignment(time_dimension.SlackVar(index))

def print_solution(data, manager, routing, assignment):  
    """Konsola sonucu printler"""
    """ print('Objective: {}'.format(assignment.ObjectiveValue()))
    print('Breaks:')
    intervals = assignment.IntervalVarContainer()
    for i in range(intervals.Size()):
        brk = intervals.Element(i)
        if brk.PerformedValue() == 1:
            print('{}: Start({}) Duration({})'.format(brk.Var().Name(),
                                                      brk.StartValue(),
                                                      brk.DurationValue()))
        else:
            print('{}: Unperformed'.format(brk.Var().Name()))
 """
    total_distance = 0
    total_load = 0
    total_time = 0
    capacity_dimension = routing.GetDimensionOrDie('Capacity')
    time_dimension = routing.GetDimensionOrDie('Time')
    big_dictionary = [] # &&&&
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        distance = 0 
        vehicle_route = [] # &&&&
        while not routing.IsEnd(index):
            load_var = capacity_dimension.CumulVar(index)
            time_var = time_dimension.CumulVar(index)
            slack_var = time_dimension.SlackVar(index)
            plan_output += ' {0} Load({1}) Time({2},{3}) Slack({4},{5}) ->'.format(
                manager.IndexToNode(index), assignment.Value(load_var),
                assignment.Min(time_var), assignment.Max(time_var),
                assignment.Min(slack_var), assignment.Max(slack_var))
            vehicle_route.append( [manager.IndexToNode(index), assignment.Value(load_var), 
            (assignment.Min(time_var), assignment.Max(time_var)),
            (assignment.Min(slack_var), assignment.Max(slack_var))]) # &&&&
            previous_index = index
            index = assignment.Value(routing.NextVar(index))
            distance += routing.GetArcCostForVehicle(previous_index, index,
                                                    vehicle_id)
        big_dictionary.append( vehicle_route)
        load_var = capacity_dimension.CumulVar(index)
        time_var = time_dimension.CumulVar(index)
        plan_output += ' {0} Load({1}) Time({2},{3})\n'.format(
            manager.IndexToNode(index), assignment.Value(load_var),
            assignment.Min(time_var), assignment.Max(time_var))
        plan_output += 'Distance of the route: {0}m\n'.format(distance)
        plan_output += 'Load of the route: {}\n'.format(
            assignment.Value(load_var))
        plan_output += 'Time of the route: {}\n'.format(
            assignment.Value(time_var))
        print(plan_output)
        total_distance += distance
        total_load += assignment.Value(load_var)
        total_time += assignment.Value(time_var)
    # print("This is big_dictionary", big_dictionary)
    routes = []
    routes_loc = []
    for i in big_dictionary:
        temp_tup = []
        temp_time = []
        temp_loc = []
        t = np.arange(0., 5., 0.2)
        for j in i:
            # print(j[0])
            temp_loc.append(j[0])
            temp_tup.append(j[0])
            temp_time.append(j[2])
        routes_loc.append(temp_loc )
        # print("routes_loc: ",routes_loc)
    
    new_routes = []
    for i,rota in enumerate(routes_loc):
        if i < len(data['ends']) : 
            addie = rota
            addie.append(data['ends'][i])
            new_routes.append(addie)
        else: 
            break
        plt.axis([0,300,0,200])
    print("new_routes: ",new_routes)
    for i,j in enumerate(new_routes):
        x_kor = []
        y_kor = []
        for x in j:
            x_kor.append(data['locations'][x][0])
            y_kor.append(data['locations'][x][1])
        plt.plot(x_kor,y_kor)        
       
   
    for i in data['locations']:
        plt.plot(i[0],i[1],'ro')
    plt.grid()
    plt.show()
    
    print('Total Distance of all routes: {0}m'.format(total_distance))
    print('Total Load of all routes: {}'.format(total_load))
    print('Total Time of all routes: {0}min'.format(total_time))


def main():
    """Programın tamamı"""
    # data problemini başlatır
    data = create_data_model()
    
    manager = pywrapcp.RoutingIndexManager(data['numlocations_'],
                                           data['num_vehicles'], #data['depot'])
                                           data['starts'], data['ends'])

    # Routing Model i yaratır
    routing = pywrapcp.RoutingModel(manager)

    
    distance_evaluator_index = routing.RegisterTransitCallback(
        functools.partial(create_distance_evaluator(data), manager))
    routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator_index)
    
    demand_evaluator_index = routing.RegisterUnaryTransitCallback(
        functools.partial(create_demand_evaluator(data), manager))
    add_capacity_constraints(routing, data, demand_evaluator_index)
    
    time_evaluator_index = routing.RegisterTransitCallback(
        functools.partial(create_time_evaluator(data), manager))
    add_time_window_constraints(routing, manager, data, time_evaluator_index)
    
    time_dimension = routing.GetDimensionOrDie('Time')
    node_visit_transit = {}
    for index in range(routing.Size()):
        node = manager.IndexToNode(index)
        node_visit_transit[index] = int(data['demands'][node] *
                                        data['time_per_demand_unit'])

    break_intervals = {}
    for v in range(data['num_vehicles']):
        vehicle_break = data['breaks'][v]
        break_intervals[v] = [
            # Başlangıç ve bitiş noktasındaki loading ve unloading için time windows ekler.
            # (unload için bir FixedDurationIntervalVar fonksiyonu daha açılır)
            # Bu windows'lar FixedDurationIntervalVar tarafından oluşturulur.
            # location'lardaki time_windows gibi start ve end time'ları çözümlemiyor.
            # windows'un genişliği vehicle_load_time ve vehicle_unload_time olarak belirlenir.
            # Süre değişkenini ayarlar. Süre 0'dan yüksek olmalıdır.
            # Eğer seçenek True ise bu süre uygulanabilir ya da uygulanmayabilir.
            # False ise bu mesafe(süre) her zaman uygulanır.
            routing.solver().FixedDurationIntervalVar(30, 100, vehicle_break[0],
                                                      vehicle_break[1],
                                                      f'Break for vehicle {v}')
        ]
        time_dimension.SetBreakIntervalsOfVehicle(break_intervals[v], v,
                                                  node_visit_transit.values())
    # Heuristik çözümü kurar

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC) 
    
    # Problemi çözmeye başlar

    assignment = routing.SolveWithParameters(search_parameters)
    
    # Problemi çözer
    # Çözüm basılabiliyorsa basılır
    if assignment:
        print_solution(data, manager, routing, assignment)
    else:
        print('No solution found!')
    
    


if __name__ == '__main__':
    main()
# [END program]
