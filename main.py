from search.algorithms import CBS, CBSState, State
from search.map import Map

def read_instances(test_instances):
    problems = []
    with open(test_instances, 'r') as file:
        for line in file:
            starts_instances = []
            goals_instances = []

            list_instance = line.split(",")
            cost = None
            for i in range(0, len(list_instance), 4):
                if i == len(list_instance) - 1:
                    cost = int(list_instance[i])
                    problems.append((starts_instances, goals_instances, cost))
                    break
                start = State(int(list_instance[i]), int(list_instance[i + 1]))
                goal = State(int(list_instance[i + 2]), int(list_instance[i + 3]))

                starts_instances.append(start)
                goals_instances.append(goal)
    return problems


def main():

    gridded_map = Map("dao-map/test_map.map")
    starts = [State(1, 1), State(5, 1)]
    goals = [State(4, 1), State(2, 1)]
    cbs_state = CBSState(gridded_map, starts, goals)
    cbs_search = CBS()
    paths, cost = cbs_search.search(cbs_state)
    if paths is not None:
        print('Solution paths encountered for the easy test: ')
        for agent, path in paths.items():
            print(agent, path)
        print()

    name_map = "dao-map/combat2.map"
    test_instances = "test-instances/instances.txt"

    problems = read_instances(test_instances)
    gridded_map = Map(name_map)
    for problem in problems:
        cbs_state = CBSState(gridded_map, problem[0], problem[1])
        cbs_search = CBS()
        _, cost = cbs_search.search(cbs_state)

        if cost != problem[2]:
            print('There was a mismatch for problem: ')
            print(problem)
            print('Expected: ', problem[2])
            print('Obtained: ', cost)
            print()
        else:
            print('Correctly Solved: ', problem[2], cost)


if __name__ == "__main__":
    main()