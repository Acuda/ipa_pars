# ipa_pars: IPA Plannung and Reasoning System:

1. Change the algorithm parameters in ipa_pars_map_analyzer/ros/launch/ipa_pars_map_analyzer_server_params.yaml to the wanted algorithm, settings and the map input file.
2. Start the action servers using the file /ros/launch/ipa_pars_map_analyzer_server.launch, which executes the ros/src/ipa_pars_map_analyzer_server.cpp, ipa_pars_map_tesselation_server.cpp, ipa_pars_map_knowledge_extractor_server.cpp, knowledge_to_yaml.py files.
3. Start an action client, which sends a goal to the map_analyzer_server, corresponding to the ParsMapAnalyzer.action message, which lies in ipa_pars_mao_analyzer/action. Or just use the file ros/launch/ipa_pars_map_analyzer_client.launch.

