#!/usr/bin/env python3
import subprocess
import os
import sys

from aal.adaptation_strategies.adaptation_strategy import AdaptationStrategy

class PrismMarkovChainStrategy(AdaptationStrategy):

    def __init__(self):
        super().__init__('markov_chain')
 
    def suggest_adaptation(self, adaptation_state):
        # Get path to the model, property from the user (from the behaviour tree)
        model_dir = adaptation_state.model_dir

        # Initially set chosen configuration to the first one provided
        possible_configs = adaptation_state.possible_configurations
        chosen_config = possible_configs[0]
        best_util = float('-inf')

        if not os.path.exists(model_dir):
            print(f"Path {model_dir} does not exist, please provide a correct path.")
            return chosen_config
        
        if not os.path.isfile(f"{model_dir}/base_model.pm"):
            print(f"File base_model.pm is missing from the given directory.")
            return chosen_config
            
        if not os.path.isfile(f"{model_dir}/property.pctl"):
            print(f"File property.pctl is missing from the given directory.")
            return chosen_config
            
        if not os.path.isfile(f"{model_dir}/required_vars.txt"):
            print(f"File required_vars.txt is missing from the given directory.")
            return chosen_config
        
        if not os.path.isfile(f"{model_dir}/utility_function.py"):
            print(f"File utility_function.py is missing from the given directory.")
            return chosen_config

        # Import the utility function from the model directory
        sys.path.append(model_dir)
        from utility_function import calculate_utility
        
        # Get path to PRISM program
        prism_bin = "~/rebet_ws/src/aal/prism-4.8.1-linux64-x86/bin/prism"

        # Check if all required parametrised PRISM variables are provided (sometimes not the case in first few iterations),
        # and in case of strings, store possible values of the string in a dictionairy
        str_vars = {}
        keys = [kv.key.lower() for kv in adaptation_state.context] + \
               [qr.qr_name.lower() for qr in adaptation_state.qrs] + \
               [param.name for param in possible_configs[0].configuration_parameters]
        
        with open(f'{model_dir}/required_vars.txt', 'r') as required_vars_file:
            for var in required_vars_file:
                split_line = var.split()
                # Regular case of variable written straight to the model
                if len(split_line) == 1:
                    if var.strip() not in keys:
                        print(f"Important context is missing: {var}")
                        return chosen_config
                # String case with possible values
                else:
                    if split_line[0] not in keys:
                        print(f"Important context is missing: {split_line[0]}")
                        return chosen_config
                    else:
                        if len(split_line[1:]) <= 1:
                            print(f"Too few possible values for string variable: {split_line[0]}")
                            return chosen_config
                        str_vars[split_line[0]] = split_line[1:]

        # Write qr-metrics and context values to the model (all assumed to be given as doubles)
        with open(f'{model_dir}/base_model.pm','r') as base_model_file:
            base_model = base_model_file.read()
        for qr in adaptation_state.qrs:
            base_model += f'\nconst double {qr.qr_name.lower()} = {qr.metric};'
        for kv in adaptation_state.context:
            base_model += f'\nconst double {kv.key.lower()} = {kv.value};'
        
        for config in possible_configs:
            # Fill in configuration parameters in model
            with open(f'{model_dir}/final_model.pm','w') as model_file:
                model_file.write(base_model)
                for param in config.configuration_parameters:
                    if param.value.type == 1:   # Case of bool
                        model_file.write(f'\nconst bool {param.name} = {str(param.value.bool_value).lower()};')
                    elif param.value.type == 2: # Case of integer
                        model_file.write(f'\nconst int {param.name} = {param.value.integer_value};')
                    elif param.value.type == 3: # Case of double
                        model_file.write(f'\nconst double {param.name} = {param.value.double_value};')
                    elif param.value.type == 4: # Case of string
                        # PRISM does not support strings, so instead write the index of the value
                        if param.value.string_value not in str_vars[param.name]:
                            print(f"Value given to string {param.name} not listed as possible valus")
                        model_file.write(f'\nconst int {param.name} = {str_vars[param.name].index(param.value.string_value)};')
                    # TODO: handle the array types as well
                
            # Run PRISM for config
            completed_process = subprocess.run(
                [f'{prism_bin} {model_dir}/final_model.pm {model_dir}/properties.pctl'],
                shell=True, capture_output=True, text=True)

            # Parse output
            prop_results = []
            output = completed_process.stdout
            print(output)
            for result_string in output.split("Result: ")[1:]:
                value_string = result_string.split()[0]
                # Depending on the property, result may be a bool, int or float
                if value_string == "true":
                    prop_results.append(True)
                elif value_string == "false":
                    prop_results.append(False)
                elif value_string.isdigit():
                     prop_results.append(int(value_string))
                else:
                     prop_results.append(float(value_string))

            # Use provided function to get utility of the adaptation, update chosen config if this one is better
            util = calculate_utility(prop_results)
            if util > best_util:
                chosen_config = config
                best_util = util

        return chosen_config
    