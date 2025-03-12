#!/usr/bin/env python3
import numpy as np
import subprocess
import os

from aal.adaptation_strategies.adaptation_strategy import AdaptationStrategy

class PrismStrategy(AdaptationStrategy):

    def __init__(self):
        super().__init__('prism')
 
    def suggest_adaptation(self, adaptation_state):
        print("\n\nstart\n\n")

        print(f'{adaptation_state.possible_configurations}')
        print(f'{adaptation_state.qrs}')
        print(f'{adaptation_state.context}')
        print(f"utility: {adaptation_state.current_utility}")

        prism_bin = "~/rebet_ws/prism-4.8.1-linux64-x86/bin/prism"
        models_path = '~/rebet_ws/src/aal/aal/aal/adaptation_strategies/models'
        full_models_path = os.path.expanduser(models_path)

        possible_configs = adaptation_state.possible_configurations
        best_config = possible_configs[0]
        best_util = float('-inf')

        with open(f'{full_models_path}/base_model.pm','r') as base_model_file:
            base_model = base_model_file.read()

        for qr in adaptation_state.qrs:
            base_model += f'\nconst double {qr.qr_name.lower()} = {qr.metric};'

        for kv in adaptation_state.context:
            base_model += f'\nconst double {kv.key.lower()} = {kv.value.lower()};'

        print(base_model)

        for config in possible_configs:
        #     speed = config.configuration_parameters[0].value.double_array_value[0]

            # TODO: consider what to do at the beginning/when the values from frog_adapt_nodes (utility_of_adaptation()) come as 0.0 or negative
            
            # TODO: What if there are multiple configurations, i.e. the rate and topic
            configname = config.configuration_parameters[0].name
            configname = configname.split(';')[0]

            # TODO: What if its not a double, idk how to know beforehand.
            configvalue = config.configuration_parameters[0].value.double_value

            # Fill in monitored/config-speicifc data into model
            with open(f'{full_models_path}/final_model.pm','w') as model_file:
                model_file.write(base_model)
                model_file.write(f'\nconst double {configname} = {configvalue};')
                
            # # Run PRISM for config
            # completed_process = subprocess.run(
            #     [f'{prism_bin} {models_path}/final_model.pm {models_path}/properties.pctl'],
            #     shell=True, capture_output=True, text=True)

            # # Parse output
            # prop_results = []
            # output = completed_process.stdout

            # print(output)

            # # Put results for each property in an array
            # for result_string in output.split("Result: ")[1:]:
            #     prop_results.append(float(result_string.split()[0]))

            # # print(prop_results)
            # util = prop_results[0] # TODO: rethink, maybe the utility can be calculated in the model as a property rather than here
            # if util > best_util:
            #     best_config = config
            #     best_util = util

        print("\n\nend\n\n")
        return best_config
    