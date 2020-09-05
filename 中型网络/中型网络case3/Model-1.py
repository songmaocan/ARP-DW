#coding=utf8
from Input_from_excel import Excel_processor
import copy
import cplex
class DW:
    def __init__(self):
        # Data input
        self.base_profit=-100000  #big profit for initial feasible solution
        data = Excel_processor(self.base_profit)
        self.node_list, self.g_number_of_nodes, self.link_list, self.g_number_of_links, self.task_list, \
        self.g_number_of_tasks, self.virtual_depot_id = data.read_links()
        # vehicle data
        self.g_number_of_vehicles=6
        self.vehicle_capacity=100
        self.vehicle_departure_time_beginning=0
        self.vehicle_arrival_time_ending=100
        self.big_M=50
        #DW parameter
        self.DW_iteration_time=400

        # DP parameters
        self.g_ending_state_vector = [None] * (self.g_number_of_vehicles + 1)
        self.g_time_dependent_state_vector=[]
        for v in range(self.g_number_of_vehicles + 1):
            self.g_time_dependent_state_vector.append([])
            intervals = self.vehicle_arrival_time_ending - self.vehicle_departure_time_beginning
            for t in range(intervals + 1):
                self.g_time_dependent_state_vector[v].append([None] * self.g_number_of_nodes)
        # beam searching：每个结点保留最多两个状态，成本最小
        self.Best_K_Size = 3

        #Column information
        self.g_Column_SST_path=[] #[[[state],[space],[time]]]
        self.g_Column_cost=[] #[cost]
        self.g_Column_task_to_path_matrix=[] #[[all tasks]]

        #Output information
        #1.record LB and UB
        #2.path information (Iteration/vehicle/SST path)
        self.optimal_SST_path=[]
        self.selected_columns=[]
        #3.dual variables(iteration/|P|+1)
        # self.dual_variables=[]
        print("Input is complete！")

    def g_Dantzig_Wolf_decomposition_method(self):

        print("Step1: Initial assignment and routing is completed!")
        #Step 1.4 Construct the RMP and
        #Iterations of RMP and SP
        task_serving_list = [0] * self.g_number_of_tasks
        task_serving_list_1=copy.copy(task_serving_list)
        selected_column_list=[]  #key
        flag=0
        UB=0
        LB=0
        while True:
            print(task_serving_list)
            flag+=1
            self.g_initialize_feasible_solution(selected_column_list,task_serving_list_1)
            for i in range(self.DW_iteration_time):
            #Step2: Solve the RMP and relaxed RMP
                self.g_initialize_RMP(task_serving_list,selected_column_list)
                self.RMP.solve()
                dual_variables=self.RMP.solution.get_dual_values()
                print(dual_variables)
                # self.dual_variables.append(dual_variables)
                print(self.RMP.solution.get_values())
                #update the dual variables
                num=0
                for task_id in range(self.g_number_of_tasks):
                    if task_serving_list[task_id]!=1:
                        task_id_in_link_list = self.task_list[task_id]
                        self.link_list[task_id_in_link_list].base_profit_for_lagrangian=dual_variables[num]*(-1)
                        num+=1
                #step 3: Solve the subproblem and generate the lower bound!
                self.g_optimal_time_dependent_dynamic_programming(0, 2)
                reduced_cost=self.g_ending_state_vector[0].VSStateVector[0].Label_cost_for_lagrangian-dual_variables[-1]


                print("reduced_cost: {}".format(reduced_cost))
                if round(reduced_cost,3)<0:
                    SST_path = []
                    SST_path.append(self.g_ending_state_vector[0].VSStateVector[0].m_visit_state_seq)
                    SST_path.append(self.g_ending_state_vector[0].VSStateVector[0].m_visit_space_seq)
                    SST_path.append(self.g_ending_state_vector[0].VSStateVector[0].m_visit_time_seq)
                    self.g_Column_SST_path.append(SST_path)

                    path_cost = self.g_ending_state_vector[0].VSStateVector[0].Primal_Label_cost
                    self.g_Column_cost.append(path_cost)

                    serving_state = self.g_ending_state_vector[0].VSStateVector[0].task_service_state  # all tasks
                    self.g_Column_task_to_path_matrix.append(serving_state)

                if round(reduced_cost,6)>=0 or i==self.DW_iteration_time-1:
                    if flag==1:
                        LB=self.RMP.solution.get_objective_value()
                        print("LB:{}".format(LB))
                    solution=self.RMP.solution.get_values()

                    max_flow = max(solution)
                    index = solution.index(max_flow)
                    selected_column_list.append(index)
                    #delete the column from column pool
                    self.optimal_SST_path.append(self.g_Column_SST_path[index])

                    print(index)
                    print(selected_column_list)
                    column_cost=self.g_Column_cost[index]
                    UB+=column_cost
                    serving_state = self.g_Column_task_to_path_matrix[index]
                    for task_id in range(self.g_number_of_tasks):
                        task_serving_list[task_id] += serving_state[task_id]
                        if task_serving_list[task_id]>1:
                            print("repeat!!!")
                            exit()
                    del self.g_Column_SST_path[index]
                    del self.g_Column_cost[index]
                    del self.g_Column_task_to_path_matrix[index]

                    """
                    serving_state=self.g_Column_task_to_path_matrix[index]
                    for task_id in range(self.g_number_of_tasks):
                        task_serving_list[task_id]+=serving_state[task_id]
                    variable_list=self.RMP.variables.get_names()
                    variable=variable_list[index]
                    lin_express = [[variable], [1]]
                    self.RMP.linear_constraints.add(lin_expr=[lin_express], senses=["E"], rhs=[1])
                    """
                    print("Flag: {}".format(flag))
                    print("iterations: {}".format(i))
                    break
            if sum(task_serving_list)==self.g_number_of_tasks or len(selected_column_list)==self.g_number_of_vehicles:
                break
        print("optimal_UB: {}".format(UB))
        print("LB:{}".format(LB))



    def g_initialize_feasible_solution(self,selected_column_list,serving_flag):
        #初始化
        for task_id in range(self.g_number_of_tasks):
            state=serving_flag[task_id]
            task_id_in_link_list = self.task_list[task_id]
            if state == 1:  # served task
                self.link_list[task_id_in_link_list].base_profit_for_lagrangian = 100000
            else:
                self.link_list[task_id_in_link_list].base_profit_for_lagrangian = -100000
        # Step1: generate a group of feasible path by LR
        # UB=0
        # serving_flag=[0]*self.g_number_of_tasks
        # task_serving_list
        for v in range(self.g_number_of_vehicles-len(selected_column_list)):
            # step 1.1 Solve the SP for vehicle v
            self.g_optimal_time_dependent_dynamic_programming(v, 2)  # 2 LR
            # Step 1.2 add the path to the Column pool
            SST_path = []
            SST_path.append(self.g_ending_state_vector[v].VSStateVector[0].m_visit_state_seq)
            SST_path.append(self.g_ending_state_vector[v].VSStateVector[0].m_visit_space_seq)
            SST_path.append(self.g_ending_state_vector[v].VSStateVector[0].m_visit_time_seq)
            self.g_Column_SST_path.append(SST_path)

            path_cost = self.g_ending_state_vector[v].VSStateVector[0].Primal_Label_cost
            self.g_Column_cost.append(path_cost)

            serving_state = self.g_ending_state_vector[v].VSStateVector[0].task_service_state  # all tasks
            self.g_Column_task_to_path_matrix.append(serving_state)
            # step 1.3 update the served tasks' multipliers
            for task_id in range(self.g_number_of_tasks):
                # 从0到任务数-1 映射到 原始的task_id，也就是路段ID
                task_id_in_link_list = self.task_list[task_id]
                if serving_state[task_id] == 1:  # served task
                    self.link_list[task_id_in_link_list].base_profit_for_lagrangian = 100000
                    serving_flag[task_id]=1
        if sum(serving_flag)!=self.g_number_of_tasks:
            print("初始化未完成！")
        else:
            print("初始化完成！")
            print(serving_flag)

    def g_initialize_RMP(self,task_serving_list,selected_column_list):
        # LP
        self.RMP = cplex.Cplex()
        # add variables y_l
        for l in range(len(self.g_Column_cost)):
            self.RMP.variables.add(obj=[self.g_Column_cost[l]], lb=[0], ub=[1],types=[self.RMP.variables.type.continuous], names=["y_{}".format(l)])
        self.variable_list = self.RMP.variables.get_names()
        # add task satisfaction cosntraints
        for task_id in range(self.g_number_of_tasks):
            if task_serving_list[task_id]!=1:
                lin_express = [[], []]
                for l in range(len(self.g_Column_cost)):
                    lin_express[0].append("y_{}".format(l))
                    lin_express[1].append(self.g_Column_task_to_path_matrix[l][task_id])
                self.RMP.linear_constraints.add(lin_expr=[lin_express], senses=["E"], rhs=[1],names=["task_satisfaction_{}".format(task_id)])

        # add flow constraint
        lin_express = [[], [1] * len(self.g_Column_cost)]
        for l in range(len(self.g_Column_cost)):
            lin_express[0].append("y_{}".format(l))
        self.RMP.linear_constraints.add(lin_expr=[lin_express], senses=["E"], rhs=[self.g_number_of_vehicles-len(selected_column_list)],names=["flow_constraint"])

        #variable range
        # for l in range(len(self.g_Column_cost)):
        #     lin_express = [["y_{}".format(l)], [1]]
        #     self.RMP.linear_constraints.add(lin_expr=[lin_express], senses=["G"], rhs=[0],names=["LB_{}".format(l)])
        #     self.RMP.linear_constraints.add(lin_expr=[lin_express], senses=["L"], rhs=[1], names=["UB_{}".format(l)])

        self.RMP.set_problem_type(self.RMP.problem_type.LP)

    def g_optimal_time_dependent_dynamic_programming(self, vehicle_id, Flag):
        self.origin_node = self.virtual_depot_id
        self.destination_node = self.virtual_depot_id
        for t in range(self.vehicle_departure_time_beginning, self.vehicle_arrival_time_ending + 1):
            for n in range(self.g_number_of_nodes):
                # 给每个时空点一个属性，并且有个更新成本的函数。
                self.g_time_dependent_state_vector[vehicle_id][t][n] = C_time_indexed_state_vector()
                self.g_time_dependent_state_vector[vehicle_id][t][n].Reset()
                self.g_time_dependent_state_vector[vehicle_id][t][n].current_time = t
                self.g_time_dependent_state_vector[vehicle_id][t][n].current_node = n

        self.g_ending_state_vector[vehicle_id] = C_time_indexed_state_vector()

        # 初始化起点状态
        element = CVSState(self.g_number_of_tasks)
        element.current_node_id = self.origin_node
        element.m_visit_space_seq.append(self.origin_node)
        element.m_visit_time_seq.append(self.vehicle_departure_time_beginning)
        element.m_visit_state_seq.append(self.vehicle_capacity)
        self.g_time_dependent_state_vector[vehicle_id][self.vehicle_departure_time_beginning][
            self.origin_node].update_state(element, Flag)
        # DP
        for t in range(self.vehicle_departure_time_beginning, self.vehicle_arrival_time_ending):
            for n in range(self.g_number_of_nodes):
                self.g_time_dependent_state_vector[vehicle_id][t][n].Sort(Flag)
                for index in range(
                        min(len(self.g_time_dependent_state_vector[vehicle_id][t][n].VSStateVector), self.Best_K_Size)):
                    pElement = self.g_time_dependent_state_vector[vehicle_id][t][n].VSStateVector[index]
                    from_node_id = pElement.current_node_id  # 起点ID
                    from_node = self.node_list[from_node_id]  # 起点
                    for i in range(from_node.outbound_size):  # 邻接点
                        to_node_id = from_node.outbound_nodes_list[i]  # 从当前点可以到达哪些点ID
                        to_node = self.node_list[to_node_id]  # 邻接点
                        link_to = from_node.outbound_links_list[i]  # 邻接路段
                        next_time = t + link_to.travel_time
                        if next_time > self.vehicle_arrival_time_ending:
                            continue
                        if to_node_id == self.destination_node:
                            new_element = CVSState(self.g_number_of_tasks)
                            new_element.my_copy(pElement)
                            new_element.current_node_id = to_node_id  # 当前点
                            # 运输弧
                            new_element.m_visit_space_seq.append(self.destination_node)
                            new_element.m_visit_time_seq.append(next_time)
                            new_element.m_visit_state_seq.append(copy.copy(pElement.m_visit_state_seq[-1]))
                            # 虚拟弧
                            new_element.m_visit_space_seq.append(self.destination_node)
                            new_element.m_visit_time_seq.append(self.vehicle_arrival_time_ending)
                            new_element.m_visit_state_seq.append(copy.copy(pElement.m_visit_state_seq[-1]))
                            # 计算成本
                            new_element.Calculate_Label_Cost(link_to)
                            self.g_ending_state_vector[vehicle_id].update_state(new_element, Flag)
                            continue
                        # 判断是不是任务路段
                        if link_to.demand == 0:  # 非任务路段
                            new_element = CVSState(self.g_number_of_tasks)
                            new_element.my_copy(pElement)
                            new_element.current_node_id = to_node_id  # 当前点
                            # 运输弧
                            new_element.m_visit_space_seq.append(to_node_id)
                            new_element.m_visit_time_seq.append(next_time)
                            new_element.m_visit_state_seq.append(copy.copy(pElement.m_visit_state_seq[-1]))
                            # 计算成本
                            new_element.Calculate_Label_Cost(link_to)
                            self.g_time_dependent_state_vector[vehicle_id][next_time][to_node_id].update_state(
                                new_element, Flag)
                            continue
                        # 判断水量是否充足
                        if pElement.m_visit_state_seq[-1] < link_to.demand:
                            continue
                        # 判断车辆受否可以访问这个任务路段
                        # Task_id是多少呢？
                        link_id = self.link_list.index(link_to)  # 从0开始
                        task_id_from_0 = self.task_list.index(link_id)
                        if pElement.task_service_state[task_id_from_0] == 1:
                            continue
                        new_element = CVSState(self.g_number_of_tasks)
                        new_element.my_copy(pElement)
                        new_element.current_node_id = to_node_id  # 当前点
                        # 运输弧
                        new_element.m_visit_space_seq.append(to_node_id)
                        new_element.m_visit_time_seq.append(next_time)
                        new_element.m_visit_state_seq.append(copy.copy(pElement.m_visit_state_seq[-1]) - link_to.demand)
                        # 修改任务访问标识，0变成1
                        new_element.task_service_state[task_id_from_0] = 1
                        # 计算成本
                        new_element.Calculate_Label_Cost(link_to)
                        self.g_time_dependent_state_vector[vehicle_id][next_time][to_node_id].update_state(new_element,
                                                                                                           Flag)
        self.g_ending_state_vector[vehicle_id].Sort(Flag)


class C_time_indexed_state_vector:
    def __init__(self):
        self.current_time=0 #时间
        self.current_node=0 #空间
        self.VSStateVector=[] #SST结点的状态
        self.state_map=[]   #水量

    def Reset(self):
        self.current_time =0
        self.current_node =0
        self.VSStateVector=[]
        self.state_map=[]

    def m_find_state_index(self, string_key):  #tring_key为水量
        if string_key in self.state_map:
            return self.state_map.index(string_key)
        else:
            return -1

    def update_state(self, element, Flag):
        string_key = element.generate_string_key()
        state_index = self.m_find_state_index(string_key)
        if state_index==-1:
            self.VSStateVector.append(element)
            self.state_map.append(string_key)
        else:
            # Flag=1,ADMM成本；Flag=2,LR成本
            if Flag==2:
                if element.Label_cost_for_lagrangian<self.VSStateVector[state_index].Label_cost_for_lagrangian:
                    self.VSStateVector[state_index]=element
            if Flag==1:
                if element.Label_cost_for_searching<self.VSStateVector[state_index].Label_cost_for_searching:
                    self.VSStateVector[state_index]=element

    def Sort(self, Flag):
        if Flag == 1:
            self.VSStateVector = sorted(self.VSStateVector, key=lambda x: x.Label_cost_for_searching)
        if Flag == 2:
            self.VSStateVector = sorted(self.VSStateVector, key=lambda x: x.Label_cost_for_lagrangian)


class CVSState:
    def __init__(self, g_number_of_tasks):
        self.current_node_id=0
        self.task_service_state = [0] * g_number_of_tasks
        # 记录SST轨迹
        self.m_visit_space_seq = []
        self.m_visit_time_seq = []
        self.m_visit_state_seq = []
        # 标号成本
        self.Primal_Label_cost=0 #一项
        self.Label_cost_for_lagrangian=0#两项
        self.Label_cost_for_searching=0 #四项

    def generate_string_key(self):
        str = self.m_visit_state_seq[-1]
        return str

    def my_copy(self, pElement):
        self.current_node_id = copy.copy(pElement.current_node_id)
        self.task_service_state = []
        self.task_service_state = copy.copy(pElement.task_service_state)
        self.m_visit_space_seq = []
        self.m_visit_space_seq = copy.copy(pElement.m_visit_space_seq)
        self.m_visit_time_seq = []
        self.m_visit_time_seq = copy.copy(pElement.m_visit_time_seq)
        self.m_visit_state_seq = []
        self.m_visit_state_seq = copy.copy(pElement.m_visit_state_seq)

        self.Primal_Label_cost= copy.copy(pElement.Primal_Label_cost)
        self.Label_cost_for_lagrangian = copy.copy(pElement.Label_cost_for_lagrangian)
        self.Label_cost_for_searching = copy.copy(pElement.Label_cost_for_searching)

    def Calculate_Label_Cost(self, link_information):
        #判断路段是不是TASK路段
        if link_information.demand==0:
            self.Primal_Label_cost +=link_information.travel_time
            self.Label_cost_for_lagrangian+=link_information.travel_time
            self.Label_cost_for_searching+=link_information.travel_time
        else:
            self.Primal_Label_cost += link_information.travel_time
            self.Label_cost_for_lagrangian += link_information.travel_time+link_information.base_profit_for_lagrangian
            self.Label_cost_for_searching += link_information.travel_time+link_information.base_profit_for_searching