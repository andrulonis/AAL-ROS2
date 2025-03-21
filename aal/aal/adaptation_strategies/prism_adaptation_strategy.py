#!/usr/bin/env python3
import numpy as np
import subprocess
import os
from importlib.machinery import SourceFileLoader
import sys

from aal.adaptation_strategies.adaptation_strategy import AdaptationStrategy

class PrismStrategy(AdaptationStrategy):

    def __init__(self):
        super().__init__('prism')
 
    def suggest_adaptation(self, adaptation_state, **kwargs):
        print("\n\nstart\n\n")
        model_dir = kwargs.get('model_dir', None)

        print(f'{adaptation_state.qrs}')
        print(f'{adaptation_state.context}')
        # print(f"utility: {adaptation_state.current_utility}")

        prism_bin = "~/rebet_ws/prism-4.8.1-linux64-x86/bin/prism"
        if model_dir != '':
            full_models_path = model_dir
        else:
            models_path = '~/rebet_ws/src/aal/aal/aal/adaptation_strategies/models'
            full_models_path = os.path.expanduser(models_path)
        
        sys.path.append(full_models_path)
        from utility_function import calculate_utility # type: ignore - Module loaded from the models dir
        
        possible_configs = adaptation_state.possible_configurations
        best_config = possible_configs[0]
        best_util = float('-inf')

        str_vars = {}
        keys =  [kv.key.lower() for kv in adaptation_state.context] + \
                [qr.qr_name.lower() for qr in adaptation_state.qrs] + \
                [param.name for param in possible_configs[0].configuration_parameters]
        
        with open(f'{full_models_path}/required_vars.txt', 'r') as required_vars_file:
            for var in required_vars_file:
                split_line = var.split()
                # Regular case of variable written straight to the model
                if len(split_line) == 1:
                    if var.strip() not in keys:
                        print(f"Important context is missing: {var}")
                        return best_config
                # String case with possible values
                else:
                    if split_line[0] not in keys:
                        print(f"Important context is missing: {split_line[0]}")
                        return best_config
                    else:
                        if len(split_line[1:]) <= 1:
                            print(f"Too few possible values for string variable: {split_line[0]}")
                            return best_config
                        str_vars[split_line[0]] = split_line[1:]


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
                        if param.value.string_value not in str_vars[param.name]:
                            print(f"Value given to string {param.name} not listed as possible valus")
                        model_file.write(f'\nconst int {param.name} = {str_vars[param.name].index(param.value.string_value)};')
                
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

            util = calculate_utility(prop_results)
            if util > best_util:
                best_config = config
                best_util = util

        print("\n\nend\n\n")
        return best_config
    