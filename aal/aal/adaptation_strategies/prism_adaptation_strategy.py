#!/usr/bin/env python3
import numpy as np
import subprocess
import os

from aal.adaptation_strategies.adaptation_strategy import AdaptationStrategy

class PrismStrategy(AdaptationStrategy):

    def __init__(self):
        super().__init__('prism')

    # TODO: add here or in the model the case when we are out of power_budget, then assign -infinity as utility so we do not allow such adapatation to be sent at all and then think if we somehow want to exclude the internal cam adaptation model checking and just do it on the external one to preserve computation time
    # TODO: Once done, move this function out to a utility_function.py and then we can say it is abstracted and replacable
    # Will want this to be not a specific function, but rather something the user can specify
    def calculate_utility(self, props):
        if not props[1]:
            return 0
        if props[3] == 1 and props[4] == 1:
            return float('-inf')
        return props[2]
 
    def suggest_adaptation(self, adaptation_state):
        print("\n\nstart\n\n")

        # print(f'{adaptation_state.qrs}')
        print(f'{adaptation_state.context}')
        # print(f"utility: {adaptation_state.current_utility}")

        prism_bin = "~/rebet_ws/prism-4.8.1-linux64-x86/bin/prism"
        models_path = '~/rebet_ws/src/aal/aal/aal/adaptation_strategies/models'
        full_models_path = os.path.expanduser(models_path)

        possible_configs = adaptation_state.possible_configurations
        best_config = possible_configs[0]
        best_util = float('-inf')

        # TODO: find better way to check if required context is present
        contextkeys = [kv.key.lower() for kv in adaptation_state.context]
        if 'power_budget' not in contextkeys or 'obstacles' not in contextkeys or 'obs_detected' not in contextkeys:
            print("Important context is missing")
            return best_config

        with open(f'{full_models_path}/base_model.pm','r') as base_model_file:
            base_model = base_model_file.read()

        for qr in adaptation_state.qrs:
            base_model += f'\nconst double {qr.qr_name.lower()} = {qr.metric};'

        for kv in adaptation_state.context:
            base_model += f'\nconst double {kv.key.lower()} = {kv.value.lower()};'
        

        for config in possible_configs:
            # Fill in configuration parameters in model
            with open(f'{full_models_path}/final_model.pm','w') as model_file:
                model_file.write(base_model)
                for param in config.configuration_parameters:
                    if param.value.type == 1:
                        model_file.write(f'\nconst bool {param.name} = {str(param.value.bool_value).lower()};')
                    elif param.value.type == 2:
                        model_file.write(f'\nconst int {param.name} = {param.value.integer_value};')
                    elif param.value.type == 3:
                        model_file.write(f'\nconst double {param.name} = {param.value.double_value};')
                    elif param.value.type == 4:
                        # PRISM does not have strings, hence for each string parameter one needs to add a special rule/handling for it
                        if (param.name == "image_topic_name"):
                            int_cam_used = param.value.string_value == "/camera/image_noisy"
                            model_file.write(f'\nconst bool int_cam_used = {str(int_cam_used).lower()};')
                
            # Run PRISM for config
            completed_process = subprocess.run(
                [f'{prism_bin} {models_path}/final_model.pm {models_path}/properties.pctl'],
                shell=True, capture_output=True, text=True)

            # Parse output
            prop_results = []
            output = completed_process.stdout

            print(output)

            # Put results for each property in an array
            for result_string in output.split("Result: ")[1:]:
                value_string = result_string.split()[0]
                if value_string == "true":
                    prop_results.append(True)
                elif value_string == "false":
                    prop_results.append(False)
                elif value_string.isdigit():
                     prop_results.append(int(value_string))
                else:
                     prop_results.append(float(value_string))

            print(f'\nResults for config with rate {config.configuration_parameters[0].value.integer_value} and topic {config.configuration_parameters[1].value.string_value}:')
            print(f'{prop_results}')

            util = self.calculate_utility(prop_results) # TODO: rethink, maybe the utility can be calculated in the model as a property rather than here or possibly make the "scientist" to provide a utility function and then just call it here, define an empty one in this file above
            if util > best_util:
                best_config = config
                best_util = util

        print("\n\nend\n\n")
        return best_config
    