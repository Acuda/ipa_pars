# ipa_pars: IPA Plannung and Reasoning System:

This provides a general planning and reasoning system to work with the Care-O-bot. 
Wiki and Readme is still in progress.

# MapAnalyzer

The MapAnalyzer creates static knowledge for action planning from a 2D map how it is used for general navigation tasks. This provides areas of interest for the robot and the possiblity to plan in its known environment.

1. Change the algorithm parameters in ipa_pars_map_analyzer/ros/launch/ipa_pars_map_analyzer_server_params.yaml to the wanted algorithm, settings and the map input file.
2. Start the action servers using the file ipa_pars_map_analyzer/ros/launch/ipa_pars_map_analyzer_server.launch, which executes the ros/src/ipa_pars_map_analyzer_server.cpp, ipa_pars_map_tesselation_server.cpp, ipa_pars_map_knowledge_extractor_server.cpp, knowledge_to_yaml.py files.
3. Start an action client, which sends a goal to the map_analyzer_server, corresponding to the ParsMapAnalyzer.action message, which lies in ipa_pars_mao_analyzer/action. Or just use the file ros/launch/ipa_pars_map_analyzer_client.launch.

The static knowledge file which is build in this progress is stored int /.ros/ipa_pars/knowledge/static-knowledge-base.yaml in the yaml format.

# Creating Dynamic Knowledge

The action knowledge the robot should have at beginning of the planning task has to be written by hand. The dynamic-knowledge is stored in /.ros/ipa_pars/knowledge/dynamic-knowledge-base.yaml.

# Create Domain File

The action planner needs a problem.pddl and a domain.pddl for logic planning. This stack provides automatical creation of the problem.pddl file from static and dynamic knowledge. The domain file, must be hand written dependent of the actions the robot needs to perform in the specific environment. How to write a new action for the robot is described in the ros Wiki.
 
# Start a planning task

1. Start the action server using the file ipa_pars_main/ros/launch/planning_server.launch, which executes the ipa_pars_main/ros/planning_server.py, planning_solver_server.py, planning_executor_server.py, knowledge_parser_server.py.
2. Start an action client, which sends a goal to the planning_server, corresponding to the LogicPlan.action message, which lies in ipa_pars_main/action.

# Start a demo:

1. Start the action client in ipa_pars_main/common/launch/planning_client.launch.
